#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# © 2025 Robotics 4.0.
# Este archivo forma parte de ejemplos y guías de simulación distribuidos bajo
# la Licencia Apache 2.0.
#
# Puedes usarlo, modificarlo y redistribuirlo libremente citando la fuente:
#     Robotics 4.0 - 2025
#
# Nota: Este código es de carácter ilustrativo, pensado para simulación con
#       Unitree MuJoCo y SDK2. No representa el producto final de Robotics 4.0.
# -----------------------------------------------------------------------------
# @file g1_arms_example.py
# @author Robotics 4.0 Team
# @version 1.0
# @brief Reproducción de rutinas de brazos y torso del G1 de 23 DoF en MuJoCo.
#
# @descripcion
#   Este script publica comandos LowCmd interpolados para ejecutar rutinas
#   articulares almacenadas en archivos JSON o TXT. Está configurado
#   exclusivamente para el orden DDS oficial del G1 de 23 DoF:
#
#       0-11  piernas
#       12    torso
#       13-17 brazo izquierdo
#       18-22 brazo derecho
#
#   Las articulaciones no incluidas en la rutina conservan la posición tomada
#   del primer LowState recibido.
#
# @uso
#   python3 g1_arms_example.py --pose <rutina.json>
#   python3 g1_arms_example.py --pose <rutina.txt> --interface lo
# -----------------------------------------------------------------------------

import argparse
import json
import math
import sys
import time
from pathlib import Path

try:
    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize,
        ChannelPublisher,
        ChannelSubscriber,
    )
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.utils.crc import CRC
    from unitree_sdk2py.utils.thread import RecurrentThread
except Exception as error:
    print("[ERROR] No se pudo importar unitree_sdk2py.")
    print("Verifica que el entorno de Unitree SDK2 Python esté instalado y activado.")
    print(f"Detalle: {error}")
    sys.exit(1)


G1_NUM_MOTOR = 23
VALID_UPPER_BODY_INDICES = set(range(12, 23))

Kp = [
    60, 60, 60, 100, 40, 40,      # pierna izquierda
    60, 60, 60, 100, 40, 40,      # pierna derecha
    60,                             # torso
    40, 40, 40, 40, 40,           # brazo izquierdo
    40, 40, 40, 40, 40,           # brazo derecho
]

Kd = [
    1, 1, 1, 2, 1, 1,             # pierna izquierda
    1, 1, 1, 2, 1, 1,             # pierna derecha
    1,                              # torso
    1, 1, 1, 1, 1,                # brazo izquierdo
    1, 1, 1, 1, 1,                # brazo derecho
]

SCRIPT_DIR = Path(__file__).resolve().parent


def find_default_joint_map():
    candidates = [
        SCRIPT_DIR / "config" / "g1_23dof_joint_map.json",
        SCRIPT_DIR / "extras" / "config" / "g1_23dof_joint_map.json",
        SCRIPT_DIR.parent / "config" / "g1_23dof_joint_map.json",
    ]

    for candidate in candidates:
        if candidate.is_file():
            return candidate

    return candidates[0]


DEFAULT_JOINT_MAP = find_default_joint_map()


class Mode:
    PR = 0
    AB = 1


def init_channel(interface):
    if interface == "lo":
        ChannelFactoryInitialize(1, "lo")
    else:
        ChannelFactoryInitialize(0, interface)


def load_json(path):
    with open(path, "r", encoding="utf-8") as file:
        return json.load(file)


def load_routine(path):
    suffix = path.suffix.lower()

    if suffix == ".json":
        return load_json(path)

    if suffix != ".txt":
        raise ValueError("Formato no compatible. Utiliza un archivo .json o .txt.")

    content = path.read_text(encoding="utf-8").strip()

    if not content:
        raise ValueError("El archivo de rutina está vacío.")

    try:
        return json.loads(content)
    except json.JSONDecodeError:
        routine = {
            "nombre_rutina": path.stem,
            "pasos": [],
        }

        for line_number, line in enumerate(content.splitlines(), start=1):
            line = line.strip()

            if not line or line.startswith("#"):
                continue

            parts = line.split()
            if len(parts) < 3:
                raise ValueError(
                    f"Línea {line_number} inválida. "
                    "Formato esperado: indice valor duracion"
                )

            try:
                joint = int(parts[0])
                value = float(parts[1])
                duration = float(parts[2])
            except ValueError as error:
                raise ValueError(
                    f"Línea {line_number} contiene datos no numéricos."
                ) from error

            routine["pasos"].append(
                {
                    "nombre": f"Paso {len(routine['pasos']) + 1}",
                    "posiciones": {str(joint): value},
                    "duracion": duration,
                }
            )

        return routine


def load_joint_map(path):
    if path is None or not path.is_file() or path.stat().st_size == 0:
        return None

    try:
        return load_json(path)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        raise ValueError(f"No se pudo leer el mapa articular {path}: {error}") from error


def indices_from_routine(routine):
    indices = set()

    for step in routine.get("pasos", []):
        for key in step.get("posiciones", {}).keys():
            try:
                indices.add(int(key))
            except (TypeError, ValueError):
                raise ValueError(f"Índice articular inválido en la rutina: {key}")

    return sorted(indices)


def upper_body_indices_from_joint_map(joint_map):
    if not joint_map:
        return []

    values = joint_map.get("upper_body_motor_indices", [])

    try:
        return sorted(set(int(value) for value in values))
    except (TypeError, ValueError) as error:
        raise ValueError(
            "El campo upper_body_motor_indices contiene valores inválidos."
        ) from error


def validate_23dof_indices(indices, source_name):
    invalid = sorted(
        index
        for index in set(indices)
        if index < 0
        or index >= G1_NUM_MOTOR
        or index not in VALID_UPPER_BODY_INDICES
    )

    if invalid:
        raise ValueError(
            f"{source_name} contiene índices no válidos para la parte superior "
            f"del G1 de 23 DoF: {invalid}. "
            "Los índices admitidos son 12-22 en el esquema DDS oficial."
        )


class PosePlayer:
    def __init__(self, controlled_indices, control_dt=0.002):
        self.num_motors = G1_NUM_MOTOR
        self.controlled_indices = sorted(set(int(i) for i in controlled_indices))
        self.controlled_index_set = set(self.controlled_indices)
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

    def init_dds(self):
        self.lowcmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher.Init()

        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.low_state_handler, 10)

    def low_state_handler(self, msg):
        self.low_state = msg

        if hasattr(msg, "mode_machine"):
            self.mode_machine = msg.mode_machine

    def wait_lowstate(self, timeout=8.0):
        print("[INFO] Esperando rt/lowstate...")
        start = time.time()

        while self.low_state is None:
            if time.time() - start > timeout:
                raise RuntimeError(
                    "No llegó rt/lowstate. Verifica que MuJoCo esté ejecutándose "
                    "con el modelo de 23 DoF."
                )
            time.sleep(0.05)

        if len(self.low_state.motor_state) < self.num_motors:
            raise RuntimeError(
                "LowState no contiene los 23 estados motores requeridos."
            )

        print("[OK] rt/lowstate recibido.")

    def initialize_from_current_state(self):
        if self.low_state is None:
            raise RuntimeError("No hay LowState para inicializar.")

        for index in range(self.num_motors):
            position = float(self.low_state.motor_state[index].q)
            self.q_init[index] = position
            self.target_pos[index] = position
            self.current_cmd_pos[index] = position

        print("[OK] Posición inicial tomada desde LowState.")

    def interpolate_position(self, q_start, q_target):
        if self.T <= 0:
            return q_target

        ratio = (
            (1.0 - math.cos(math.pi * (self.t / self.T))) / 2.0
            if self.t < self.T
            else 1.0
        )
        return q_start + (q_target - q_start) * ratio

    def low_cmd_write(self):
        if self.low_state is None:
            return

        cmd = unitree_hg_msg_dds__LowCmd_()
        cmd.mode_pr = Mode.PR
        cmd.mode_machine = self.mode_machine

        for index in range(self.num_motors):
            cmd.motor_cmd[index].mode = 1
            cmd.motor_cmd[index].dq = 0.0
            cmd.motor_cmd[index].tau = 0.0
            cmd.motor_cmd[index].kp = Kp[index]
            cmd.motor_cmd[index].kd = Kd[index]

            if index in self.controlled_index_set:
                q_start = self.q_init[index]
                q_target = self.target_pos[index]
                commanded_position = self.interpolate_position(
                    q_start,
                    q_target,
                )
            else:
                commanded_position = self.current_cmd_pos[index]

            cmd.motor_cmd[index].q = commanded_position
            self.current_cmd_pos[index] = commanded_position

        cmd.crc = self.crc.Crc(cmd)
        self.lowcmd_publisher.Write(cmd)

        self.t += self.control_dt

    def start_writer(self):
        if self.writer_thread is not None:
            raise RuntimeError("El hilo LowCmd ya está en ejecución.")

        self.writer_thread = RecurrentThread(
            interval=self.control_dt,
            target=self.low_cmd_write,
            name="g1_23dof_arms_writer",
        )
        self.writer_thread.Start()
        print("[OK] Writer LowCmd iniciado.")

    def move_to(self, updates, duration):
        if self.low_state is None:
            raise RuntimeError("LowState no recibido.")

        for index in self.controlled_indices:
            self.q_init[index] = self.current_cmd_pos[index]

        for key, value in updates.items():
            index = int(key)

            if index not in self.controlled_index_set:
                print(
                    f"[WARN] Índice {index} no está habilitado para esta rutina. "
                    "Se ignora."
                )
                continue

            self.target_pos[index] = float(value)

        self.T = max(float(duration), 0.001)
        self.t = 0.0

        wait_start = time.time()
        max_wait = self.T + 2.0

        while self.t < self.T:
            if time.time() - wait_start > max_wait:
                raise RuntimeError(
                    "El hilo de control no completó la interpolación dentro "
                    "del tiempo esperado."
                )
            time.sleep(self.control_dt)

        self.t = self.T
        time.sleep(max(self.control_dt, 0.002))

    def play_routine(self, routine):
        name = routine.get("nombre_rutina", "routine")
        steps = routine.get("pasos", [])

        if not isinstance(steps, list):
            raise ValueError("El campo 'pasos' debe ser una lista.")

        print(f"\n[INFO] Ejecutando rutina: {name}")
        print(f"[INFO] Pasos: {len(steps)}")

        for step in steps:
            step_name = step.get("nombre", "Paso")
            duration = float(step.get("duracion", 1.0))
            raw_positions = step.get("posiciones", {})

            if not isinstance(raw_positions, dict):
                raise ValueError(
                    f"Las posiciones del paso '{step_name}' deben ser un objeto."
                )

            updates = {}

            for key, value in raw_positions.items():
                index = int(key)
                updates[index] = float(value)

            print(
                f"  -> {step_name} | dur={duration:.3f}s | "
                f"joints={sorted(updates.keys())}"
            )
            self.move_to(updates, duration)

        print("[OK] Rutina finalizada.")

    def hold_current_pose(self, repeat=50, delay=0.02):
        final_cmd = unitree_hg_msg_dds__LowCmd_()
        final_cmd.mode_pr = Mode.PR
        final_cmd.mode_machine = self.mode_machine

        for index in range(self.num_motors):
            final_cmd.motor_cmd[index].mode = 1
            final_cmd.motor_cmd[index].q = self.current_cmd_pos[index]
            final_cmd.motor_cmd[index].dq = 0.0
            final_cmd.motor_cmd[index].tau = 0.0
            final_cmd.motor_cmd[index].kp = Kp[index]
            final_cmd.motor_cmd[index].kd = Kd[index]

        final_cmd.crc = self.crc.Crc(final_cmd)

        for _ in range(repeat):
            self.lowcmd_publisher.Write(final_cmd)
            time.sleep(delay)

    def stop(self):
        # El hilo debe detenerse antes de publicar el comando final para evitar
        # que ambas rutas escriban simultáneamente sobre rt/lowcmd.
        if self.writer_thread is not None:
            try:
                self.writer_thread.Wait()
                print("[INFO] Writer LowCmd detenido.")
            finally:
                self.writer_thread = None

        if self.lowcmd_publisher is not None and self.low_state is not None:
            self.hold_current_pose()

        print("[INFO] Programa terminado.")


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Ejecuta una rutina JSON o TXT de brazos/torso del G1 de 23 DoF "
            "en MuJoCo."
        )
    )
    parser.add_argument(
        "--pose",
        required=True,
        help="Ruta al archivo JSON o TXT de la rutina.",
    )
    parser.add_argument(
        "--interface",
        default="lo",
        help="Interfaz DDS. En simulación local normalmente: lo",
    )
    parser.add_argument(
        "--joint-map",
        default=str(DEFAULT_JOINT_MAP),
        help=(
            "Mapa articular opcional. Si no existe, se utilizan los índices "
            "presentes en la rutina."
        ),
    )
    parser.add_argument(
        "--control-dt",
        type=float,
        default=0.002,
        help="Periodo del hilo de control en segundos.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=8.0,
        help="Tiempo máximo para esperar rt/lowstate.",
    )
    args = parser.parse_args()

    pose_path = Path(args.pose).expanduser().resolve()

    if not pose_path.is_file():
        print(f"[ERROR] No existe la rutina: {pose_path}")
        sys.exit(1)

    try:
        routine = load_routine(pose_path)
        routine_indices = indices_from_routine(routine)
        validate_23dof_indices(routine_indices, "La rutina")

        joint_map_path = Path(args.joint_map).expanduser()
        joint_map = load_joint_map(joint_map_path)
        map_indices = upper_body_indices_from_joint_map(joint_map)

        if map_indices:
            validate_23dof_indices(map_indices, "El mapa articular")
            controlled_indices = sorted(
                set(map_indices) | set(routine_indices)
            )
        else:
            controlled_indices = routine_indices

        if not controlled_indices:
            raise ValueError("No hay índices controlables en la rutina.")

    except (OSError, TypeError, ValueError, json.JSONDecodeError) as error:
        print(f"[ERROR] {error}")
        sys.exit(1)

    print("\n[CONFIGURACIÓN]")
    print(f"Interfaz: {args.interface}")
    print(f"Número de motores: {G1_NUM_MOTOR}")
    print(f"Rutina: {pose_path}")
    print(f"Índices controlados: {controlled_indices}")
    print("")

    init_channel(args.interface)

    player = PosePlayer(
        controlled_indices=controlled_indices,
        control_dt=args.control_dt,
    )
    player.init_dds()

    exit_code = 0

    try:
        player.wait_lowstate(timeout=args.timeout)
        player.initialize_from_current_state()

        input(
            "Verifica que el robot esté estable y sin obstáculos. "
            "Presiona Enter para ejecutar..."
        )

        player.start_writer()
        player.play_routine(routine)

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C detectado. Interrumpiendo.")
    except Exception as error:
        print(f"[ERROR] {error}")
        exit_code = 1
    finally:
        player.stop()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
