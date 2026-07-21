#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Selector numerado de rutinas JSON para Unitree G1 23 DoF en MuJoCo.
#
# Uso:
#   1. Abrir MuJoCo con G1 23 DoF.
#   2. Ejecutar:
#        python3 g1_23dof_mujoco_selector.py
#      o:
#        python3 g1_23dof_mujoco_selector.py lo
#
# El menú se construye automáticamente leyendo los .json de:
#   Libreria de Poses/scripts/poses_json/
#
# Comandos:
#   número = ejecutar rutina
#   l      = listar rutinas otra vez
#   x      = salir y sostener última postura
# -----------------------------------------------------------------------------

import time
import sys
import json
import re
from pathlib import Path

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread


# Aunque el modelo operativo sea G1 23 DoF, el LowCmd mantiene slots tipo G1.
G1_NUM_MOTOR = 29

# Ganancias base heredadas del selector de G1.
Kp = [
    60, 60, 60, 100, 40, 40,
    60, 60, 60, 100, 40, 40,
    60, 40, 40,
    40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40
]

Kd = [
    1, 1, 1, 2, 1, 1,
    1, 1, 1, 2, 1, 1,
    1, 1, 1,
    1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1
]


class Mode:
    PR = 0
    AB = 1


class G123DoFMujocoSelector:
    """
    Selector de rutinas JSON para G1 23 DoF.

    Joints activos para poses:
      12: waist_yaw_joint
      15: left_shoulder_pitch_joint
      16: left_shoulder_roll_joint
      17: left_shoulder_yaw_joint
      18: left_elbow_joint
      19: left_wrist_roll_joint
      22: right_shoulder_pitch_joint
      23: right_shoulder_roll_joint
      24: right_shoulder_yaw_joint
      25: right_elbow_joint
      26: right_wrist_roll_joint

    Se excluyen:
      13, 14, 20, 21, 27, 28
    porque corresponden a grados extra del modelo 29 DoF.
    """

    def __init__(self, poses_dir: Path, control_dt: float = 0.002):
        self.poses_dir = poses_dir
        self.control_dt = control_dt
        self.crc = CRC()

        self.lowcmd_publisher_ = None
        self.lowstate_subscriber = None
        self.low_state = None
        self.mode_machine_ = 0
        self._writer_thread = None

        self.controlled_joints = [
            12,
            15, 16, 17, 18, 19,
            22, 23, 24, 25, 26
        ]

        self.excluded_29dof_only_joints = [
            13, 14, 20, 21, 27, 28
        ]

        self.target_pos = {i: 0.0 for i in range(G1_NUM_MOTOR)}
        self.q_init = {i: 0.0 for i in range(G1_NUM_MOTOR)}
        self.current_cmd_pos = {i: 0.0 for i in range(G1_NUM_MOTOR)}

        # Posiciones de retención para joints no controlados.
        # Esto evita mandar piernas o grados extra a cero de forma brusca.
        self.hold_pos = {i: 0.0 for i in range(G1_NUM_MOTOR)}

        self.t = 0.0
        self.T = 1.0

    # ---------------------------------------------------------
    # Comunicación MuJoCo
    # ---------------------------------------------------------

    def Init(self):
        self.lowcmd_publisher_ = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher_.Init()

        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg
        if hasattr(msg, "mode_machine"):
            self.mode_machine_ = msg.mode_machine

    def wait_lowstate(self, timeout=8.0):
        print("[INFO] Esperando rt/lowstate...")
        start = time.time()

        while self.low_state is None:
            if time.time() - start > timeout:
                print("[ERROR] LowState no recibido dentro del timeout.")
                print("Verifica que MuJoCo esté abierto y que estés usando interface lo.")
                return False
            time.sleep(0.05)

        for i in range(G1_NUM_MOTOR):
            try:
                q0 = float(self.low_state.motor_state[i].q)
            except Exception:
                q0 = 0.0

            self.hold_pos[i] = q0
            self.q_init[i] = q0
            self.target_pos[i] = q0
            self.current_cmd_pos[i] = q0

        print("\n[POSE INICIAL REAL - JOINTS CONTROLADOS]")
        for j in self.controlled_joints:
            print(f'"{j}": {float(self.low_state.motor_state[j].q):.6f},')
        print("[FIN POSE INICIAL]\n")

        return True

    # ---------------------------------------------------------
    # Interpolación y envío LowCmd
    # ---------------------------------------------------------

    def interpolate_position(self, q_init, q_target):
        if self.T <= 0:
            return q_target

        s = max(0.0, min(self.t / self.T, 1.0))
        return q_init + (q_target - q_init) * s

    def LowCmdWrite(self):
        if self.low_state is None:
            return

        cmd = unitree_hg_msg_dds__LowCmd_()
        cmd.mode_pr = Mode.PR
        cmd.mode_machine = self.mode_machine_

        for i in range(G1_NUM_MOTOR):
            cmd.motor_cmd[i].mode = 1
            cmd.motor_cmd[i].kp = Kp[i]
            cmd.motor_cmd[i].kd = Kd[i]
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].tau = 0.0

            if i in self.controlled_joints:
                q0 = self.q_init.get(i, self.current_cmd_pos.get(i, 0.0))
                q1 = self.target_pos.get(i, q0)
                cmd.motor_cmd[i].q = self.interpolate_position(q0, q1)
            else:
                # Mantener el resto en la postura inicial real.
                # No se fuerzan piernas ni joints extra a cero.
                cmd.motor_cmd[i].q = self.hold_pos.get(i, 0.0)

        cmd.crc = self.crc.Crc(cmd)
        self.lowcmd_publisher_.Write(cmd)

        self.t += self.control_dt

    def StartWriter(self):
        if self._writer_thread is None:
            self._writer_thread = RecurrentThread(
                self.control_dt,
                target=self.LowCmdWrite,
                name="g1_23dof_lowcmd_writer"
            )
            self._writer_thread.Start()
            print("[INFO] LowCmd writer iniciado.")
        else:
            print("[WARN] Writer ya estaba corriendo.")

    def move_to(self, updates: dict, duration: float = 1.0):
        if self.low_state is None:
            raise RuntimeError("LowState no recibido. No se puede mover con seguridad.")

        for j in self.controlled_joints:
            self.q_init[j] = float(
                self.current_cmd_pos.get(j, self.low_state.motor_state[j].q)
            )

        new_targets = {
            j: float(self.current_cmd_pos.get(j, self.low_state.motor_state[j].q))
            for j in self.controlled_joints
        }

        ignored = []

        for k, v in updates.items():
            try:
                jidx = int(k)
                value = float(v)
            except Exception:
                continue

            if jidx in self.controlled_joints:
                new_targets[jidx] = value
            elif jidx in self.excluded_29dof_only_joints:
                ignored.append(jidx)

        if ignored:
            print(f"[INFO] Joints 29 DoF ignorados para G1 23 DoF: {sorted(set(ignored))}")

        for j in self.controlled_joints:
            self.target_pos[j] = new_targets[j]

        self.T = float(duration) if float(duration) > 0 else 0.001
        self.t = 0.0

        while self.t < self.T:
            time.sleep(self.control_dt)

        self.t = self.T

        for j in self.controlled_joints:
            self.current_cmd_pos[j] = self.target_pos[j]
            self.q_init[j] = self.target_pos[j]

        time.sleep(max(self.control_dt, 0.002))

    # ---------------------------------------------------------
    # Carga y ejecución de rutinas
    # ---------------------------------------------------------

    def load_routine(self, filepath: Path):
        if not filepath.is_file():
            raise FileNotFoundError(f"Archivo no encontrado: {filepath}")

        with open(filepath, "r", encoding="utf-8") as f:
            routine = json.load(f)

        return routine

    def PlayRoutine(self, routine: dict):
        name = routine.get("nombre_rutina", "routine")
        pasos = routine.get("pasos", [])

        print("\n" + "=" * 72)
        print(f"[INFO] Ejecutando rutina: {name}")
        print(f"[INFO] Total de pasos: {len(pasos)}")
        print("=" * 72)

        for idx, paso in enumerate(pasos, 1):
            pname = paso.get("nombre", f"Paso {idx}")
            dur = float(paso.get("duracion", 1.0))
            raw_pos = paso.get("posiciones", {})

            updates = {}

            for k, v in raw_pos.items():
                try:
                    joint_idx = int(k)
                    updates[joint_idx] = float(v)
                except Exception:
                    continue

            active_updates = {
                k: v for k, v in updates.items()
                if k in self.controlled_joints
            }

            print(
                f"  -> {idx:02d}. {pname} | "
                f"dur={dur:.2f}s | joints={sorted(active_updates.keys())}"
            )

            self.move_to(updates, duration=dur)

        print("[INFO] Rutina finalizada.")

    # ---------------------------------------------------------
    # Catálogo automático
    # ---------------------------------------------------------

    @staticmethod
    def extract_number(path: Path):
        """
        Extrae el prefijo numérico del archivo:
          0_pose_segura.json -> 0
          11 confusion.json   -> 11
          12_siu.json         -> 12
        """
        match = re.match(r"^\s*(\d+)", path.stem)
        if match:
            return int(match.group(1))
        return None

    @staticmethod
    def natural_key(path: Path):
        n = G123DoFMujocoSelector.extract_number(path)
        if n is None:
            return (9999, path.name.lower())
        return (n, path.name.lower())

    def build_catalog(self):
        if not self.poses_dir.is_dir():
            return []

        files = sorted(self.poses_dir.glob("*.json"), key=self.natural_key)

        catalog = []
        used_numbers = set()
        fallback_number = 1

        for path in files:
            number = self.extract_number(path)

            if number is None or number in used_numbers:
                while fallback_number in used_numbers:
                    fallback_number += 1
                number = fallback_number

            used_numbers.add(number)

            catalog.append({
                "number": number,
                "path": path,
                "name": path.stem
            })

        return catalog

    def print_menu(self):
        catalog = self.build_catalog()

        print("\n" + "=" * 72)
        print("SELECTOR DE RUTINAS G1 23 DoF - MUJOCO")
        print("=" * 72)
        print(f"Carpeta de rutinas: {self.poses_dir}")
        print("Escribe el número de la rutina para ejecutarla.")
        print("Comandos: l = listar | x = salir")
        print("-" * 72)

        if not catalog:
            print("[WARN] No hay archivos .json en la carpeta de poses.")
            print("-" * 72)
            return

        for item in catalog:
            print(f"{item['number']:02d}. {item['path'].name}")

        print("-" * 72)

    def find_catalog_item(self, number: int):
        for item in self.build_catalog():
            if item["number"] == number:
                return item
        return None

    def execute_item(self, item):
        path = item["path"]

        print("\n" + "=" * 72)
        print(f"[RUN] #{item['number']:02d} -> {path.name}")
        print(f"[FILE] {path}")
        print("=" * 72)

        try:
            routine = self.load_routine(path)
            self.PlayRoutine(routine)
        except Exception as e:
            print(f"[ERROR] No se pudo ejecutar {path.name}: {e}")

    def selector_loop(self):
        self.print_menu()

        while True:
            choice = input("\nNúmero de rutina / l / x: ").strip().lower()

            if choice == "":
                continue

            if choice == "l":
                self.print_menu()
                continue

            if choice == "x":
                print("[INFO] Saliendo del selector.")
                return

            if not choice.isdigit():
                print("[WARN] Entrada inválida. Usa un número, l o x.")
                continue

            number = int(choice)
            item = self.find_catalog_item(number)

            if item is None:
                print(f"[WARN] No existe rutina con número: {number}")
                continue

            self.execute_item(item)

    # ---------------------------------------------------------
    # Cierre
    # ---------------------------------------------------------

    def StopAndShutdown(self, repeat: int = 50, delay: float = 0.02):
        if self.low_state is None:
            print("[WARN] LowState no recibido. Intentando cierre de todas formas.")

        final_cmd = unitree_hg_msg_dds__LowCmd_()
        final_cmd.mode_pr = Mode.PR
        final_cmd.mode_machine = self.mode_machine_

        for i in range(G1_NUM_MOTOR):
            final_cmd.motor_cmd[i].mode = 1

            if i in self.controlled_joints:
                final_q = self.current_cmd_pos.get(i, self.hold_pos.get(i, 0.0))
            else:
                final_q = self.hold_pos.get(i, 0.0)

            final_cmd.motor_cmd[i].q = final_q
            final_cmd.motor_cmd[i].dq = 0.0
            final_cmd.motor_cmd[i].tau = 0.0
            final_cmd.motor_cmd[i].kp = Kp[i]
            final_cmd.motor_cmd[i].kd = Kd[i]

        final_cmd.crc = self.crc.Crc(final_cmd)

        for _ in range(repeat):
            self.lowcmd_publisher_.Write(final_cmd)
            time.sleep(delay)

        try:
            if self._writer_thread is not None:
                self._writer_thread.Wait()
                self._writer_thread = None
        except Exception:
            pass

        print("[INFO] Cierre completado. Última postura sostenida.")


def main():
    script_dir = Path(__file__).resolve().parent
    default_poses_dir = script_dir.parent / "poses_json"

    interface = "lo"
    poses_dir = default_poses_dir

    # Uso simple:
    #   python3 g1_23dof_mujoco_selector.py
    #   python3 g1_23dof_mujoco_selector.py lo
    #   python3 g1_23dof_mujoco_selector.py lo /ruta/a/poses_json
    if len(sys.argv) >= 2:
        interface = sys.argv[1]

    if len(sys.argv) >= 3:
        poses_dir = Path(sys.argv[2]).expanduser().resolve()

    print("WARNING: Asegúrate de que MuJoCo G1 23 DoF esté corriendo antes de ejecutar.")
    print(f"[INFO] Interface: {interface}")
    print(f"[INFO] Poses dir: {poses_dir}")
    input("Presiona Enter para continuar...")

    ChannelFactoryInitialize(1, interface)

    selector = G123DoFMujocoSelector(
        poses_dir=poses_dir,
        control_dt=0.002
    )

    selector.Init()

    if not selector.wait_lowstate(timeout=8.0):
        return

    selector.StartWriter()

    try:
        selector.selector_loop()
    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C detectado. Cerrando selector...")
    finally:
        selector.StopAndShutdown()
        print("[INFO] Programa terminado.")


if __name__ == "__main__":
    main()
