#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Selector físico de rutinas JSON para Unitree G1 23 DoF.
#
# Uso en robot físico:
#   python3 g1_23dof_physical_selector.py eth0
#
# Estructura esperada:
#   23_arm_sdk/
#   ├── g1_23dof_physical_selector.py
#   Codigos simulacion
#   └── 23 dof/
#        └── poses_json/
#           ├── 0_pose_segura.json
#           ├── 1_saludo_derecha.json
#       └    ── ...
#
# Características:
#   - Usa rt/arm_sdk para robot físico.
#   - Activa arm_sdk mediante motor_cmd[29].q = 1.
#   - Controla solo joints activos del G1 23 DoF:
#       12, 15, 16, 17, 18, 19, 22, 23, 24, 25, 26
#   - Ignora joints extra del modelo 29 DoF:
#       13, 14, 20, 21, 27, 28
#   - Catálogo dinámico: cualquier JSON agregado a poses_json aparece en el menú.
#   - Corrige pausas: pasos repetidos se ejecutan como hold real, sin retroceso.
# -----------------------------------------------------------------------------

import argparse
import csv
import json
import math
import os
import re
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC


G1_NUM_MOTOR = 30
K_NOT_USED_JOINT = 29

ACTIVE_JOINTS = [
    12,
    15, 16, 17, 18, 19,
    22, 23, 24, 25, 26
]

EXCLUDED_29DOF_ONLY_JOINTS = [
    13, 14, 20, 21, 27, 28
]

JOINT_NAMES = {
    12: "waist_yaw_joint",
    15: "left_shoulder_pitch_joint",
    16: "left_shoulder_roll_joint",
    17: "left_shoulder_yaw_joint",
    18: "left_elbow_joint",
    19: "left_wrist_roll_joint",
    22: "right_shoulder_pitch_joint",
    23: "right_shoulder_roll_joint",
    24: "right_shoulder_yaw_joint",
    25: "right_elbow_joint",
    26: "right_wrist_roll_joint",
}


class G123DoFPhysicalSelector:
    def __init__(
        self,
        interface: str,
        poses_dir: Path,
        control_dt: float = 0.02,
        kp: float = 60.0,
        kd: float = 1.5,
        min_duration: float = 0.35,
        hold_epsilon: float = 1e-4,
        max_abs_rad: float = 2.8,
        log_csv: bool = True,
    ):
        self.interface = interface
        self.poses_dir = poses_dir
        self.control_dt = float(control_dt)
        self.kp = float(kp)
        self.kd = float(kd)
        self.min_duration = float(min_duration)
        self.hold_epsilon = float(hold_epsilon)
        self.max_abs_rad = float(max_abs_rad)

        self.lock = threading.RLock()
        self.crc = CRC()

        self.low_state = None
        self.first_update_low_state = False
        self.arm_sdk_publisher = None
        self.lowstate_subscriber = None

        self.writer_thread = None
        self.writer_stop = threading.Event()

        self.current_cmd_pos = {j: 0.0 for j in ACTIVE_JOINTS}
        self.motion_start_pos = {j: 0.0 for j in ACTIVE_JOINTS}
        self.motion_target_pos = {j: 0.0 for j in ACTIVE_JOINTS}

        self.motion_active = False
        self.motion_start_time = 0.0
        self.motion_duration = 1.0

        self.csv_file = None
        self.csv_writer = None
        self.sample_count = 0
        self.log_csv = log_csv

    # ---------------------------------------------------------
    # Inicialización DDS
    # ---------------------------------------------------------

    def init_dds(self):
        print(f"[INFO] Inicializando ChannelFactory en interfaz: {self.interface}")
        ChannelFactoryInitialize(0, self.interface)

        self.arm_sdk_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.arm_sdk_publisher.Init()

        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.low_state_handler, 10)

        if self.log_csv:
            log_dir = Path.cwd() / "logs_physical"
            log_dir.mkdir(parents=True, exist_ok=True)

            self.csv_file = open(
                log_dir / f"g1_23dof_physical_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
                mode="w",
                newline="",
                encoding="utf-8"
            )
            self.csv_writer = csv.writer(self.csv_file)

            header = ["timestamp"]
            for j in ACTIVE_JOINTS:
                header.extend([f"q_{j}_{JOINT_NAMES.get(j, '')}", f"tau_{j}"])
            self.csv_writer.writerow(header)

    def low_state_handler(self, msg: LowState_):
        with self.lock:
            self.low_state = msg
            self.first_update_low_state = True

        if self.log_csv and self.csv_writer is not None:
            self.sample_count += 1
            if self.sample_count >= 500:
                self.sample_count = 0
                row = [datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")]
                for j in ACTIVE_JOINTS:
                    try:
                        row.extend([msg.motor_state[j].q, msg.motor_state[j].tau_est])
                    except Exception:
                        row.extend(["", ""])
                self.csv_writer.writerow(row)

    def wait_lowstate(self, timeout: float = 8.0):
        print("[INFO] Esperando rt/lowstate...")
        start = time.time()

        while not self.first_update_low_state:
            if time.time() - start > timeout:
                raise RuntimeError("No se recibió rt/lowstate. Revisa interfaz, red y estado del robot.")
            time.sleep(0.05)

        with self.lock:
            for j in ACTIVE_JOINTS:
                q = float(self.low_state.motor_state[j].q)
                self.current_cmd_pos[j] = q
                self.motion_start_pos[j] = q
                self.motion_target_pos[j] = q

        print("\n[POSE INICIAL REAL - JOINTS CONTROLADOS]")
        for j in ACTIVE_JOINTS:
            print(f'  "{j}": {self.current_cmd_pos[j]: .6f},  # {JOINT_NAMES.get(j, "")}')
        print("[FIN POSE INICIAL]\n")

    # ---------------------------------------------------------
    # Hilo de escritura
    # ---------------------------------------------------------

    @staticmethod
    def smooth_ratio(ratio: float):
        ratio = max(0.0, min(float(ratio), 1.0))
        return 0.5 - 0.5 * math.cos(math.pi * ratio)

    def start_writer(self):
        if self.writer_thread is not None and self.writer_thread.is_alive():
            print("[WARN] Writer ya estaba activo.")
            return

        self.writer_stop.clear()
        self.writer_thread = threading.Thread(
            target=self.writer_loop,
            name="g1_23dof_physical_writer",
            daemon=True
        )
        self.writer_thread.start()
        print("[INFO] Hilo de escritura físico iniciado.")

    def writer_loop(self):
        next_time = time.monotonic()

        while not self.writer_stop.is_set():
            self.low_cmd_write()

            next_time += self.control_dt
            sleep_time = next_time - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_time = time.monotonic()

    def low_cmd_write(self):
        with self.lock:
            if self.low_state is None:
                return

            now = time.monotonic()

            if self.motion_active:
                elapsed = now - self.motion_start_time
                ratio = elapsed / max(self.motion_duration, 1e-6)
                s = self.smooth_ratio(ratio)

                q_cmd = {}
                for j in ACTIVE_JOINTS:
                    q0 = self.motion_start_pos[j]
                    q1 = self.motion_target_pos[j]
                    q_cmd[j] = q0 + (q1 - q0) * s

                if ratio >= 1.0:
                    for j in ACTIVE_JOINTS:
                        self.current_cmd_pos[j] = self.motion_target_pos[j]
                        q_cmd[j] = self.motion_target_pos[j]
                    self.motion_active = False
            else:
                q_cmd = dict(self.current_cmd_pos)

            cmd = unitree_hg_msg_dds__LowCmd_()

            # Activación obligatoria de arm_sdk para robot físico.
            cmd.motor_cmd[K_NOT_USED_JOINT].q = 1.0

            for j in ACTIVE_JOINTS:
                cmd.motor_cmd[j].q = float(q_cmd[j])
                cmd.motor_cmd[j].dq = 0.0
                cmd.motor_cmd[j].tau = 0.0
                cmd.motor_cmd[j].kp = self.kp
                cmd.motor_cmd[j].kd = self.kd

            cmd.crc = self.crc.Crc(cmd)
            self.arm_sdk_publisher.Write(cmd)

    # ---------------------------------------------------------
    # Movimiento y hold
    # ---------------------------------------------------------

    def build_target_from_step(self, raw_positions: dict):
        target = dict(self.current_cmd_pos)
        ignored_excluded = []
        ignored_unknown = []

        for k, v in raw_positions.items():
            try:
                idx = int(k)
                value = float(v)
            except Exception:
                ignored_unknown.append(k)
                continue

            if abs(value) > self.max_abs_rad:
                raise ValueError(
                    f"Valor fuera de límite conservador en joint {idx}: {value} rad. "
                    f"Límite actual: ±{self.max_abs_rad} rad."
                )

            if idx in ACTIVE_JOINTS:
                target[idx] = value
            elif idx in EXCLUDED_29DOF_ONLY_JOINTS:
                ignored_excluded.append(idx)
            else:
                ignored_unknown.append(idx)

        if ignored_excluded:
            print(f"[INFO] Joints 29 DoF ignorados para G1 23 DoF: {sorted(set(ignored_excluded))}")

        if ignored_unknown:
            print(f"[WARN] Joints desconocidos ignorados: {ignored_unknown}")

        return target

    def hold_current_command(self, duration: float, label: str = "hold"):
        duration = max(float(duration), self.min_duration)

        with self.lock:
            self.motion_active = False

        print(f"  [HOLD] {label} durante {duration:.2f}s")
        time.sleep(duration)

    def move_to_target(self, target: dict, duration: float, label: str = "paso"):
        duration = max(float(duration), self.min_duration)

        with self.lock:
            deltas = [abs(target[j] - self.current_cmd_pos[j]) for j in ACTIVE_JOINTS]
            max_delta = max(deltas) if deltas else 0.0

            if max_delta <= self.hold_epsilon:
                self.motion_active = False
                hold_needed = True
            else:
                hold_needed = False
                self.motion_start_pos = dict(self.current_cmd_pos)
                self.motion_target_pos = dict(target)
                self.motion_duration = duration
                self.motion_start_time = time.monotonic()
                self.motion_active = True

        if hold_needed:
            self.hold_current_command(duration, label)
            return

        start = time.time()
        max_wait = duration + 2.0

        while True:
            with self.lock:
                active = self.motion_active

            if not active:
                break

            if time.time() - start > max_wait:
                print(f"[WARN] Timeout en {label}. Se sostiene última posición comandada.")
                with self.lock:
                    self.motion_active = False
                break

            time.sleep(self.control_dt)

        with self.lock:
            for j in ACTIVE_JOINTS:
                self.current_cmd_pos[j] = target[j]

    # ---------------------------------------------------------
    # Rutinas
    # ---------------------------------------------------------

    def load_routine(self, filepath: Path):
        if not filepath.is_file():
            raise FileNotFoundError(f"Archivo no encontrado: {filepath}")

        with open(filepath, "r", encoding="utf-8") as f:
            routine = json.load(f)

        pasos = routine.get("pasos", [])
        if not isinstance(pasos, list) or not pasos:
            raise ValueError("La rutina no contiene una lista válida de pasos.")

        return routine

    def play_routine(self, routine: dict):
        name = routine.get("nombre_rutina", "rutina")
        pasos = routine.get("pasos", [])

        print("\n" + "=" * 72)
        print(f"[INFO] Ejecutando rutina física: {name}")
        print(f"[INFO] Pasos: {len(pasos)}")
        print("=" * 72)

        for i, paso in enumerate(pasos, start=1):
            pname = paso.get("nombre", f"Paso {i}")
            dur = float(paso.get("duracion", 1.0))
            raw = paso.get("posiciones", {})

            if not isinstance(raw, dict):
                print(f"[WARN] {pname}: posiciones inválidas. Se omite.")
                continue

            target = self.build_target_from_step(raw)

            active_changed = [
                j for j in ACTIVE_JOINTS
                if abs(target[j] - self.current_cmd_pos[j]) > self.hold_epsilon
            ]

            print(
                f"  -> {i:02d}. {pname} | dur={max(dur, self.min_duration):.2f}s | "
                f"joints_activos={active_changed if active_changed else 'hold'}"
            )

            self.move_to_target(target, duration=dur, label=pname)

        print("[INFO] Rutina finalizada. Última postura sostenida.")

    # ---------------------------------------------------------
    # Catálogo dinámico
    # ---------------------------------------------------------

    @staticmethod
    def extract_number(path: Path):
        match = re.match(r"^\s*(\d+)", path.stem)
        if match:
            return int(match.group(1))
        return None

    @staticmethod
    def sort_key(path: Path):
        n = G123DoFPhysicalSelector.extract_number(path)
        if n is None:
            return (9999, path.name.lower())
        return (n, path.name.lower())

    def build_catalog(self):
        if not self.poses_dir.is_dir():
            return []

        files = sorted(self.poses_dir.glob("*.json"), key=self.sort_key)
        catalog = []
        used = set()
        fallback = 1

        for path in files:
            number = self.extract_number(path)

            if number is None or number in used:
                while fallback in used:
                    fallback += 1
                number = fallback

            used.add(number)
            catalog.append({
                "number": number,
                "path": path,
                "name": path.name
            })

        return catalog

    def print_menu(self):
        catalog = self.build_catalog()

        print("\n" + "=" * 72)
        print("SELECTOR FÍSICO G1 23 DoF - RUTINAS JSON")
        print("=" * 72)
        print(f"Carpeta de rutinas: {self.poses_dir}")
        print("Número = ejecutar | l = listar | x = salir seguro")
        print("-" * 72)

        if not catalog:
            print("[WARN] No hay archivos .json en poses_json.")
            print("-" * 72)
            return

        for item in catalog:
            print(f"{item['number']:02d}. {item['name']}")

        print("-" * 72)

    def find_item(self, number: int):
        for item in self.build_catalog():
            if item["number"] == number:
                return item
        return None

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
                print("[WARN] Entrada inválida.")
                continue

            number = int(choice)
            item = self.find_item(number)

            if item is None:
                print(f"[WARN] No existe rutina con número {number}.")
                continue

            print("\n" + "=" * 72)
            print(f"[SELECCIÓN] #{item['number']:02d} -> {item['name']}")
            print(f"[ARCHIVO] {item['path']}")
            print("=" * 72)

            confirm = input("Ejecutar en robot físico? Escribe 's' para confirmar: ").strip().lower()
            if confirm not in ("s", "si", "sí", "y", "yes"):
                print("[INFO] Ejecución cancelada.")
                continue

            try:
                routine = self.load_routine(item["path"])
                self.play_routine(routine)
            except KeyboardInterrupt:
                print("\n[INFO] Ctrl+C durante rutina. Se sostiene última postura comandada.")
                with self.lock:
                    self.motion_active = False
            except Exception as e:
                print(f"[ERROR] No se pudo ejecutar {item['name']}: {e}")

    # ---------------------------------------------------------
    # Salida segura
    # ---------------------------------------------------------

    def find_safe_pose_file(self):
        candidates = []

        for path in self.poses_dir.glob("*.json"):
            name = path.name.lower()
            if name.startswith("0") or "pose_segura" in name or "segura" in name:
                candidates.append(path)

        if not candidates:
            return None

        return sorted(candidates, key=self.sort_key)[0]

    def move_to_safe_pose_if_available(self):
        safe_path = self.find_safe_pose_file()

        if safe_path is None:
            print("[WARN] No se encontró pose segura. No se moverá a safe pose antes de liberar.")
            return

        print(f"[INFO] Moviendo a pose segura antes de liberar: {safe_path.name}")

        try:
            routine = self.load_routine(safe_path)
            self.play_routine(routine)
            self.hold_current_command(0.5, "hold final pose segura")
        except Exception as e:
            print(f"[WARN] No se pudo ejecutar pose segura final: {e}")

    def stop_writer(self):
        self.writer_stop.set()

        if self.writer_thread is not None:
            self.writer_thread.join(timeout=1.0)
            self.writer_thread = None

    def release_control(self):
        print("[INFO] Liberando arm_sdk...")

        self.stop_writer()

        if self.low_state is None:
            print("[WARN] Sin low_state. No se puede liberar con estado medido.")
            return

        cmd = unitree_hg_msg_dds__LowCmd_()

        with self.lock:
            for j in ACTIVE_JOINTS:
                try:
                    q_now = float(self.low_state.motor_state[j].q)
                except Exception:
                    q_now = float(self.current_cmd_pos.get(j, 0.0))

                cmd.motor_cmd[j].q = q_now
                cmd.motor_cmd[j].dq = 0.0
                cmd.motor_cmd[j].tau = 0.0
                cmd.motor_cmd[j].kp = 0.0
                cmd.motor_cmd[j].kd = 0.0

            cmd.motor_cmd[K_NOT_USED_JOINT].q = 0.0
            cmd.crc = self.crc.Crc(cmd)

        for _ in range(20):
            self.arm_sdk_publisher.Write(cmd)
            time.sleep(0.02)

        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None

        print("[INFO] Control liberado.")

    def shutdown(self, safe_on_exit: bool = True):
        try:
            if safe_on_exit:
                self.move_to_safe_pose_if_available()
        finally:
            self.release_control()


def auto_resolve_poses_dir(user_dir: str = None):
    if user_dir:
        return Path(user_dir).expanduser().resolve()

    script_dir = Path(__file__).resolve().parent
    cwd = Path.cwd()

    candidates = [
        script_dir / "poses_json",
        script_dir / "poses json",
        cwd / "poses_json",
        cwd / "poses json",
        script_dir.parent / "poses_json",
    ]

    for d in candidates:
        if d.is_dir() and any(d.glob("*.json")):
            return d.resolve()

    # Carpeta por defecto, aunque todavía esté vacía.
    return (script_dir / "poses_json").resolve()


def main():
    parser = argparse.ArgumentParser(
        description="Selector físico de rutinas JSON para Unitree G1 23 DoF."
    )
    parser.add_argument("interface", help="Interfaz de red del robot. Ej: eth0")
    parser.add_argument("--poses-dir", default=None, help="Carpeta con rutinas .json")
    parser.add_argument("--control-dt", type=float, default=0.02)
    parser.add_argument("--kp", type=float, default=60.0)
    parser.add_argument("--kd", type=float, default=1.5)
    parser.add_argument("--min-duration", type=float, default=0.35)
    parser.add_argument("--hold-epsilon", type=float, default=1e-4)
    parser.add_argument("--max-abs-rad", type=float, default=2.8)
    parser.add_argument("--no-safe-on-exit", action="store_true")
    parser.add_argument("--no-log", action="store_true")
    args = parser.parse_args()

    poses_dir = auto_resolve_poses_dir(args.poses_dir)

    print("\n" + "=" * 72)
    print("ADVERTENCIA DE EJECUCIÓN FÍSICA")
    print("=" * 72)
    print("1. El robot debe estar detenido y estable.")
    print("2. No debe haber personas u obstáculos cerca de brazos y torso.")
    print("3. Prueba primero 0_pose_segura.json.")
    print("4. Este script usa rt/arm_sdk y controla solo torso yaw + brazos 23 DoF.")
    print(f"5. Interfaz: {args.interface}")
    print(f"6. Carpeta de poses: {poses_dir}")
    print("=" * 72)

    confirm = input("Escribe 'ENTIENDO' para habilitar el selector físico: ").strip()
    if confirm != "ENTIENDO":
        print("[INFO] Cancelado por el usuario.")
        return

    selector = G123DoFPhysicalSelector(
        interface=args.interface,
        poses_dir=poses_dir,
        control_dt=args.control_dt,
        kp=args.kp,
        kd=args.kd,
        min_duration=args.min_duration,
        hold_epsilon=args.hold_epsilon,
        max_abs_rad=args.max_abs_rad,
        log_csv=not args.no_log,
    )

    try:
        selector.init_dds()
        selector.wait_lowstate(timeout=8.0)
        selector.start_writer()
        selector.selector_loop()
    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C detectado. Iniciando salida segura.")
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        selector.shutdown(safe_on_exit=not args.no_safe_on_exit)
        print("[INFO] Programa terminado.")


if __name__ == "__main__":
    main()
