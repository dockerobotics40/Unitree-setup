#!/usr/bin/env python3
import argparse
import json
import sys
import time
from pathlib import Path
from datetime import datetime

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
except Exception as e:
    print("[ERROR] No se pudo importar unitree_sdk2py.")
    print("Verifica que el entorno de Unitree SDK2 Python esté instalado/activado.")
    print(f"Detalle: {e}")
    sys.exit(1)


SCRIPT_DIR = Path(__file__).resolve().parent
POSES_ROOT = SCRIPT_DIR.parent
DEFAULT_CONFIG = POSES_ROOT / "config" / "g1_23dof_joint_map.json"
DEFAULT_OUTPUT_DIR = POSES_ROOT / "poses_json"


class LowStateReader:
    def __init__(self):
        self.low_state = None
        self.last_time = None

    def handler(self, msg: LowState_):
        self.low_state = msg
        self.last_time = time.time()


def init_channel(interface: str):
    if interface == "lo":
        ChannelFactoryInitialize(1, "lo")
    else:
        ChannelFactoryInitialize(0, interface)


def load_joint_map(path: Path):
    if not path.is_file() or path.stat().st_size == 0:
        return None

    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return None


def parse_indices(text: str):
    values = []

    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        values.append(int(item))

    return sorted(set(values))


def resolve_capture_indices(args):
    if args.indices:
        return parse_indices(args.indices), {}

    joint_map = load_joint_map(Path(args.joint_map).expanduser())

    if joint_map:
        indices = joint_map.get("upper_body_motor_indices", [])
        name_by_index = {}

        for j in joint_map.get("joints", []):
            idx = j.get("controlled_index")
            if idx is not None:
                name_by_index[int(idx)] = j.get("joint_name", f"joint_{idx}")

        if indices:
            return sorted(set(int(i) for i in indices)), name_by_index

    if args.capture_all:
        return list(range(args.num_motors)), {}

    print("[ERROR] No se pudieron resolver los índices a capturar.")
    print("Opciones:")
    print("  1. Ejecuta primero inspect_mujoco_model_23dof.py")
    print("  2. O usa --indices 12,13,14,...")
    print("  3. O usa --capture-all")
    sys.exit(1)


def get_motor_q(low_state, idx: int):
    try:
        return float(low_state.motor_state[idx].q)
    except Exception:
        raise RuntimeError(f"No se pudo leer motor_state[{idx}].q")


def snapshot_positions(low_state, indices):
    positions = {}

    for idx in indices:
        positions[str(idx)] = round(get_motor_q(low_state, idx), 6)

    return positions


def print_snapshot(positions, name_by_index):
    print("\n[POSE ACTUAL]")
    for k in sorted(positions.keys(), key=lambda x: int(x)):
        idx = int(k)
        name = name_by_index.get(idx, "")
        suffix = f"  # {name}" if name else ""
        print(f'  "{k}": {positions[k]: .6f},{suffix}')
    print("[FIN POSE ACTUAL]\n")


def build_routine(args, steps):
    return {
        "nombre_rutina": args.routine_name,
        "fecha_creacion": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "robot": "unitree_g1",
        "modelo": "g1_23dof",
        "requiere_robot_quieto": True,
        "descripcion": args.description,
        "numero_pasos": len(steps),
        "pasos": steps
    }


def save_routine(args, steps):
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    filename = f"{args.routine_name}.json"
    output_path = output_dir / filename

    if output_path.exists() and not args.overwrite:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = output_dir / f"{args.routine_name}_{stamp}.json"

    routine = build_routine(args, steps)

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(routine, f, indent=2, ensure_ascii=False)

    print(f"\n[OK] Rutina guardada en:")
    print(output_path)
    print("")


def wait_lowstate(reader: LowStateReader, timeout: float):
    print("[INFO] Esperando rt/lowstate...")

    start = time.time()

    while reader.low_state is None:
        if time.time() - start > timeout:
            print("[ERROR] No llegó rt/lowstate dentro del timeout.")
            print("Verifica que MuJoCo esté corriendo y que la interfaz sea correcta.")
            sys.exit(1)
        time.sleep(0.05)

    print("[OK] rt/lowstate recibido.")


def main():
    parser = argparse.ArgumentParser(
        description="Captura poses actuales del G1 23 DoF desde rt/lowstate."
    )
    parser.add_argument("--interface", default="lo", help="Interfaz DDS. En simulación local normalmente: lo")
    parser.add_argument("--num-motors", type=int, default=29)
    parser.add_argument("--joint-map", default=str(DEFAULT_CONFIG))
    parser.add_argument("--indices", default="", help="Índices a capturar separados por coma. Ej: 12,13,14")
    parser.add_argument("--capture-all", action="store_true", help="Captura todos los motores 0..num_motors-1")
    parser.add_argument("--routine-name", default="pose_capturada")
    parser.add_argument("--description", default="Rutina capturada manualmente desde MuJoCo.")
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args()

    indices, name_by_index = resolve_capture_indices(args)

    print("\n[CONFIGURACIÓN]")
    print(f"Interface: {args.interface}")
    print(f"Num motors: {args.num_motors}")
    print(f"Índices capturados: {indices}")
    print("")

    init_channel(args.interface)

    reader = LowStateReader()
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(reader.handler, 10)

    wait_lowstate(reader, args.timeout)

    steps = []

    print("\n[USO]")
    print("  c  capturar paso actual")
    print("  p  imprimir pose actual sin guardar")
    print("  s  guardar rutina JSON")
    print("  q  salir")
    print("")

    while True:
        cmd = input("capture> ").strip().lower()

        if cmd == "p":
            positions = snapshot_positions(reader.low_state, indices)
            print_snapshot(positions, name_by_index)

        elif cmd == "c":
            default_name = f"Paso {len(steps) + 1}"
            step_name = input(f"Nombre del paso [{default_name}]: ").strip()
            if not step_name:
                step_name = default_name

            dur_text = input("Duración del paso en segundos [1.0]: ").strip()
            if not dur_text:
                duration = 1.0
            else:
                try:
                    duration = float(dur_text)
                except Exception:
                    print("[WARN] Duración inválida. Usando 1.0 s.")
                    duration = 1.0

            positions = snapshot_positions(reader.low_state, indices)
            print_snapshot(positions, name_by_index)

            steps.append({
                "nombre": step_name,
                "posiciones": positions,
                "duracion": duration
            })

            print(f"[OK] Paso capturado. Total pasos: {len(steps)}")

        elif cmd == "s":
            if not steps:
                print("[WARN] No hay pasos capturados. Usa c primero.")
                continue

            save_routine(args, steps)

        elif cmd == "q":
            print("[INFO] Saliendo.")
            break

        else:
            print("[WARN] Comando no reconocido. Usa c, p, s o q.")


if __name__ == "__main__":
    main()
