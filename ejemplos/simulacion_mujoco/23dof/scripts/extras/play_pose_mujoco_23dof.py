#!/usr/bin/env python3
import argparse
import json
import sys
import time
from pathlib import Path

try:
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    from unitree_sdk2py.utils.crc import CRC
    from unitree_sdk2py.utils.thread import RecurrentThread
except Exception as e:
    print("[ERROR] No se pudo importar unitree_sdk2py.")
    print("Verifica que el entorno de Unitree SDK2 Python esté instalado/activado.")
    print(f"Detalle: {e}")
    sys.exit(1)


SCRIPT_DIR = Path(__file__).resolve().parent
POSES_ROOT = SCRIPT_DIR.parent
DEFAULT_JOINT_MAP = POSES_ROOT / "config" / "g1_23dof_joint_map.json"


class Mode:
    PR = 0
    AB = 1


def init_channel(interface: str):
    if interface == "lo":
        ChannelFactoryInitialize(1, "lo")
    else:
        ChannelFactoryInitialize(0, interface)


def load_json(path: Path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def load_joint_map(path: Path):
    if not path.is_file() or path.stat().st_size == 0:
        return None

    try:
        return load_json(path)
    except Exception:
        return None


def indices_from_routine(routine):
    indices = set()

    for paso in routine.get("pasos", []):
        for k in paso.get("posiciones", {}).keys():
            try:
                indices.add(int(k))
            except Exception:
                pass

    return sorted(indices)


def upper_body_indices_from_joint_map(joint_map):
    if not joint_map:
        return []

    values = joint_map.get("upper_body_motor_indices", [])
    return sorted(set(int(x) for x in values))


def make_gains(num_motors, controlled_indices):
    kp = []
    kd = []

    controlled = set(controlled_indices)

    for i in range(num_motors):
        if i in controlled:
            kp.append(40.0)
            kd.append(1.0)
        else:
            kp.append(20.0)
            kd.append(0.5)

    return kp, kd


class PosePlayer:
    def __init__(self, num_motors, controlled_indices, control_dt=0.002):
        self.num_motors = int(num_motors)
        self.controlled_indices = sorted(set(int(x) for x in controlled_indices))
        self.control_dt = float(control_dt)

        self.crc = CRC()
        self.lowcmd_publisher = None
        self.lowstate_subscriber = None
        self.low_state = None
        self.mode_machine = 0
        self.writer_thread = None

        self.target_pos = {i: 0.0 for i in range(self.num_motors)}
        self.q_init = {i: 0.0 for i in range(self.num_motors)}
        self.current_cmd_pos = {i: 0.0 for i in range(self.num_motors)}

        self.t = 0.0
        self.T = 1.0

        self.kp, self.kd = make_gains(self.num_motors, self.controlled_indices)

    def init_dds(self):
        self.lowcmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher.Init()

        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.low_state_handler, 10)

    def low_state_handler(self, msg: LowState_):
        self.low_state = msg

        if hasattr(msg, "mode_machine"):
            self.mode_machine = msg.mode_machine

    def wait_lowstate(self, timeout=8.0):
        print("[INFO] Esperando rt/lowstate...")
        start = time.time()

        while self.low_state is None:
            if time.time() - start > timeout:
                raise RuntimeError("No llegó rt/lowstate. Verifica que MuJoCo esté corriendo.")
            time.sleep(0.05)

        print("[OK] rt/lowstate recibido.")

    def initialize_from_current_state(self):
        if self.low_state is None:
            raise RuntimeError("No hay low_state para inicializar.")

        for i in range(self.num_motors):
            try:
                q = float(self.low_state.motor_state[i].q)
            except Exception:
                q = 0.0

            self.q_init[i] = q
            self.target_pos[i] = q
            self.current_cmd_pos[i] = q

        print("[OK] Posición inicial tomada desde low_state.")

    def interpolate_position(self, q0, q1):
        if self.T <= 0:
            return q1

        s = max(0.0, min(self.t / self.T, 1.0))
        return q0 + (q1 - q0) * s

    def low_cmd_write(self):
        if self.low_state is None:
            return

        cmd = unitree_hg_msg_dds__LowCmd_()
        cmd.mode_pr = Mode.PR
        cmd.mode_machine = self.mode_machine

        for i in range(self.num_motors):
            try:
                cmd.motor_cmd[i].mode = 1
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].tau = 0.0
                cmd.motor_cmd[i].kp = self.kp[i]
                cmd.motor_cmd[i].kd = self.kd[i]

                if i in self.controlled_indices:
                    q0 = self.q_init.get(i, self.current_cmd_pos.get(i, 0.0))
                    q1 = self.target_pos.get(i, q0)
                    cmd.motor_cmd[i].q = self.interpolate_position(q0, q1)
                else:
                    cmd.motor_cmd[i].q = self.current_cmd_pos.get(i, 0.0)

            except Exception:
                continue

        cmd.crc = self.crc.Crc(cmd)
        self.lowcmd_publisher.Write(cmd)

        self.t += self.control_dt

    def start_writer(self):
        if self.writer_thread is None:
            self.writer_thread = RecurrentThread(
                self.control_dt,
                target=self.low_cmd_write,
                name="g1_23dof_pose_writer"
            )
            self.writer_thread.Start()
            print("[OK] Writer LowCmd iniciado.")

    def move_to(self, updates, duration):
        if self.low_state is None:
            raise RuntimeError("LowState no recibido.")

        for i in self.controlled_indices:
            self.q_init[i] = float(self.current_cmd_pos.get(i, self.low_state.motor_state[i].q))

        for k, v in updates.items():
            idx = int(k)
            value = float(v)

            if idx not in self.controlled_indices:
                print(f"[WARN] Índice {idx} no está en controlled_indices. Se ignora.")
                continue

            self.target_pos[idx] = value

        self.T = max(float(duration), 0.001)
        self.t = 0.0

        while self.t < self.T:
            time.sleep(self.control_dt)

        self.t = self.T

        for i in self.controlled_indices:
            self.current_cmd_pos[i] = self.target_pos[i]
            self.q_init[i] = self.target_pos[i]

        time.sleep(max(self.control_dt, 0.002))

    def play_routine(self, routine):
        name = routine.get("nombre_rutina", "routine")
        pasos = routine.get("pasos", [])

        print(f"\n[INFO] Ejecutando rutina: {name}")
        print(f"[INFO] Pasos: {len(pasos)}")

        for paso in pasos:
            pname = paso.get("nombre", "Paso")
            duration = float(paso.get("duracion", 1.0))
            raw_positions = paso.get("posiciones", {})

            updates = {}

            for k, v in raw_positions.items():
                try:
                    idx = int(k)
                    updates[idx] = float(v)
                except Exception:
                    print(f"[WARN] Posición inválida ignorada: {k}: {v}")

            print(f"  -> {pname} | dur={duration:.3f}s | joints={sorted(updates.keys())}")
            self.move_to(updates, duration)

        print("[OK] Rutina finalizada.")

    def hold_final_pose(self, repeat=50, delay=0.02):
        final_cmd = unitree_hg_msg_dds__LowCmd_()
        final_cmd.mode_pr = Mode.PR
        final_cmd.mode_machine = self.mode_machine

        for i in range(self.num_motors):
            try:
                final_cmd.motor_cmd[i].mode = 1
                final_cmd.motor_cmd[i].q = self.current_cmd_pos.get(i, 0.0)
                final_cmd.motor_cmd[i].dq = 0.0
                final_cmd.motor_cmd[i].tau = 0.0
                final_cmd.motor_cmd[i].kp = self.kp[i]
                final_cmd.motor_cmd[i].kd = self.kd[i]
            except Exception:
                continue

        final_cmd.crc = self.crc.Crc(final_cmd)

        for _ in range(repeat):
            self.lowcmd_publisher.Write(final_cmd)
            time.sleep(delay)

    def stop(self):
        try:
            self.hold_final_pose()
        except Exception:
            pass

        if self.writer_thread is not None:
            try:
                self.writer_thread.Wait()
            except Exception:
                pass

            self.writer_thread = None

        print("[INFO] Programa terminado.")


def main():
    parser = argparse.ArgumentParser(description="Ejecuta una rutina JSON del G1 23 DoF en MuJoCo.")
    parser.add_argument("--pose", required=True, help="Ruta al archivo JSON de la rutina.")
    parser.add_argument("--interface", default="lo", help="Interfaz DDS. En simulación local normalmente: lo")
    parser.add_argument("--num-motors", type=int, default=29)
    parser.add_argument("--joint-map", default=str(DEFAULT_JOINT_MAP))
    parser.add_argument("--control-dt", type=float, default=0.002)
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args()

    pose_path = Path(args.pose).expanduser().resolve()

    if not pose_path.is_file():
        print(f"[ERROR] No existe la rutina: {pose_path}")
        sys.exit(1)

    routine = load_json(pose_path)

    routine_indices = indices_from_routine(routine)
    joint_map = load_joint_map(Path(args.joint_map).expanduser())
    map_indices = upper_body_indices_from_joint_map(joint_map)

    if map_indices:
        controlled_indices = sorted(set(map_indices) | set(routine_indices))
    else:
        controlled_indices = routine_indices

    if not controlled_indices:
        print("[ERROR] No hay índices controlables en la rutina.")
        sys.exit(1)

    print("\n[CONFIGURACIÓN]")
    print(f"Interface: {args.interface}")
    print(f"Num motors: {args.num_motors}")
    print(f"Rutina: {pose_path}")
    print(f"Controlled indices: {controlled_indices}")
    print("")

    init_channel(args.interface)

    player = PosePlayer(
        num_motors=args.num_motors,
        controlled_indices=controlled_indices,
        control_dt=args.control_dt
    )

    player.init_dds()

    try:
        player.wait_lowstate(timeout=args.timeout)
        player.initialize_from_current_state()

        input("Verifica que el robot esté estable y sin obstáculos. Presiona Enter para ejecutar...")

        player.start_writer()
        player.play_routine(routine)

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C detectado. Interrumpiendo.")
    except Exception as e:
        print(f"[ERROR] {e}")
        sys.exit(1)
    finally:
        player.stop()


if __name__ == "__main__":
    main()
