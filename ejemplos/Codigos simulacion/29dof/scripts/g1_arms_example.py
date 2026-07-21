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
#       Unitree mujoco y SDK2. No representa el producto final de Robotics 4.0.
# -----------------------------------------------------------------------------
# @file g1_low_level_example.py
# @author Sofía Milagros Castaño Vanegas - Robotics 4.0 Team
# @date 2025-09-21
# @version 1.0
# @brief Ejemplo de control bajo nivel de G1 en simulación para brazos
#
# @descripcion
#   Este script demuestra cómo enviar comandos LowCmd interpolados al robot G1
#   dentro del simulador Unitree Mujoco. Permite ejecutar secuencias de prueba
#   de brazos a partir de rutinas embebidas o cargadas desde archivo (.txt/.json).
#
# @requisitos
#   - Unitree mujoco instalado y configurado.
#   - SDK2 de Unitree (C++/Python) instalado en el sistema.
#   - Simulador G1 corriendo con `enable_elastic_band = 1`.
#
# @uso
#   python3 g1_arm_example.py
# -----------------------------------------------------------------------------
import time
import sys
import math
import json
import os
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

import numpy as np

# ------------------ parameters / gains ------------------
G1_NUM_MOTOR = 29

Kp = [
    60, 60, 60, 100, 40, 40,      # legs
    60, 60, 60, 100, 40, 40,      # legs
    60, 40, 40,                   # waist
    40, 40, 40, 40,  40, 40, 40,  # arms
    40, 40, 40, 40,  40, 40, 40   # arms
]

Kd = [
    1, 1, 1, 2, 1, 1,     # legs
    1, 1, 1, 2, 1, 1,     # legs
    1, 1, 1,              # waist
    1, 1, 1, 1, 1, 1, 1,  # arms
    1, 1, 1, 1, 1, 1, 1   # arms 
]

# ------------------ joint indices ------------------
class G1JointIndex:
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleB = 4
    LeftAnkleRoll = 5
    LeftAnkleA = 5
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleB = 10
    RightAnkleRoll = 11
    RightAnkleA = 11
    WaistYaw = 12
    WaistRoll = 13        # NOTE: may be locked depending on config
    WaistA = 13
    WaistPitch = 14
    WaistB = 14
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28

# ------------------ modes ------------------
class Mode:
    PR = 0  # pitch-roll series
    AB = 1  # parallel

# ------------------ main class ------------------
class Custom:
    def __init__(self, control_dt: float = 0.002):
        self.control_dt = control_dt  # 2 ms default
        self.crc = CRC()

        # publisher/subscriber placeholders (se crean en Init)
        self.lowcmd_publisher_ = None
        self.lowstate_subscriber = None

        # low-level state
        self.low_state = None
        self.mode_machine_ = 0

        # thread for sending low cmds continuously
        self._writer_thread = None

        # interpolation state
        # target positions for all joints (default 0)
        self.target_pos = {i: 0.0 for i in range(G1_NUM_MOTOR)}
        # q_init used at start of each interpolation (updated in move_to)
        self.q_init = {i: 0.0 for i in range(G1_NUM_MOTOR)}
        self.t = 0.0
        self.T = 1.0

        # which joints are considered "arm joints"
        self.arm_joints = list(range(G1JointIndex.LeftShoulderPitch, G1JointIndex.RightWristYaw + 1))

        # pre-create a low_cmd object to avoid reallocations (optional)
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()

    def Init(self):
        # Publisher
        self.lowcmd_publisher_ = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher_.Init()

        # Subscriber
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg
        # capture mode_machine once
        if hasattr(msg, "mode_machine"):
            self.mode_machine_ = msg.mode_machine

    # ---- interpolation helper ----
    def interpolate_position(self, q_init, q_target):
        """Half-cosine smoothing (0->1) based on self.t/self.T"""
        if self.T <= 0:
            return q_target
        ratio = (1 - math.cos(math.pi * (self.t / self.T))) / 2.0 if self.t < self.T else 1.0
        return q_init + (q_target - q_init) * ratio

    # ---- main low command writer (runs in recurrent thread) ----
    def LowCmdWrite(self):
        """
        This function is called periodically by the RecurrentThread.
        It composes a LowCmd where:
          - arm joints are interpolated from q_init -> target_pos using half-cos
          - non-arm joints (legs, waist) are commanded to 0
        """
        if self.low_state is None:
            return

        cmd = unitree_hg_msg_dds__LowCmd_()
        cmd.mode_pr = Mode.PR
        cmd.mode_machine = self.mode_machine_

        # fill per-joint fields
        for i in range(G1_NUM_MOTOR):
            # enable
            cmd.motor_cmd[i].mode = 1
            # PD gains
            cmd.motor_cmd[i].kp = Kp[i]
            cmd.motor_cmd[i].kd = Kd[i]
            # feedforward/velocity
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].tau = 0.0

            if i in self.arm_joints:
                q_init = self.q_init.get(i, 0.0)
                q_target = self.target_pos.get(i, q_init)
                pos = self.interpolate_position(q_init, q_target)
                cmd.motor_cmd[i].q = pos
            else:
                # legs + waist forced to zero
                cmd.motor_cmd[i].q = 0.0

        cmd.crc = self.crc.Crc(cmd)
        self.lowcmd_publisher_.Write(cmd)

        # advance interpolation time
        self.t += self.control_dt

    # ---- high-level motion API ----
    def move_to(self, updates: dict, duration: float = 1.0):
        """
        updates: dict mapping joint_index (int) -> q_target (float).
                 Only arm joints in 'updates' will be used; other indices ignored.
        duration: seconds for the interpolation
        This method blocks until interpolation completes.
        """
        # need low_state to sample q_init
        if self.low_state is None:
            raise RuntimeError("LowState not received yet — cannot move safely.")

        # capture initial positions for all arm joints from current low_state
        for j in self.arm_joints:
            try:
                self.q_init[j] = self.low_state.motor_state[j].q
            except Exception:
                # fallback to 0.0 if reading fails
                self.q_init[j] = 0.0

        # set targets (only for provided joint indices)
        for k, v in updates.items():
            if isinstance(k, str):
                jidx = int(k)
            else:
                jidx = int(k)
            # only accept if it's an arm joint, otherwise ignore
            if jidx in self.arm_joints:
                self.target_pos[jidx] = float(v)

        # interpolation parameters
        self.T = float(duration) if duration > 0 else 0.0
        self.t = 0.0

        # Wait until interpolation finished (thread should be running)
        while self.t < self.T:
            time.sleep(self.control_dt)
        # ensure final target reached (set t > T and send one command)
        self.t = self.T
        # give writer one cycle to push final pos
        time.sleep(max(self.control_dt, 0.002))

    def PlayRoutine(self, routine: dict):
        """
        routine: dict that follows your JSON structure:
          { "nombre_rutina": "...", "pasos": [ {"nombre":.., "posiciones": { "15":.. }, "duracion":.. }, ... ] }
        Executes steps sequentially using move_to.
        """
        name = routine.get("nombre_rutina", "routine")
        print(f"[INFO] Ejecutando rutina: {name}")
        for paso in routine.get("pasos", []):
            pname = paso.get("nombre", "step")
            dur = float(paso.get("duracion", 1.0))
            raw_pos = paso.get("posiciones", {})
            # convert keys to ints, keep only arm joints (others ignored but legs will remain 0)
            updates = {}
            for k, v in raw_pos.items():
                try:
                    idx = int(k)
                except:
                    continue
                if idx in self.arm_joints:
                    updates[idx] = float(v)
            print(f"  -> {pname} dur={dur}s update_joints={list(updates.keys())}")
            self.move_to(updates, duration=dur)
        print("[INFO] Rutina finalizada.")

    # ---- thread control ----
    def StartWriter(self):
        if self._writer_thread is None:
            self._writer_thread = RecurrentThread(self.control_dt, target=self.LowCmdWrite, name="lowcmd_writer")
            self._writer_thread.Start()
            print("[INFO] LowCmd writer thread started.")
        else:
            print("[WARN] Writer thread already running.")

    def StopAndShutdown(self, repeat: int = 50, delay: float = 0.02):
        """
        Stop the writer thread and send a final safe posture to all joints.
        """
        if self._writer_thread is not None:
            try:
                self._writer_thread.Wait()
            except Exception:
                pass
            self._writer_thread = None
            print("[INFO] Writer thread stopped.")

        if self.low_state is None:
            print("[WARNING] LowState not received yet. Will still attempt to send shutdown commands.")

        # --- posiciones finales predefinidas ---
        final_positions = {
            15: 0.29383397102355957,
            16: 0.22691965103149414,
            17: -0.02549433708190918,
            18: 0.9673745632171631,
            19: 0.07939767837524414,
            20: 0.022057533264160156,
            21: 0.002953767776489258,
            22: 0.29171276092529297,
            23: -0.22536182403564453,
            24: 0.030755281448364258,
            25: 0.9688725471496582,
            26: -0.1466531753540039,
            27: 0.028631210327148438,
            28: 0.008535385131835938,
            12: -0.0033248066902160645,
            13: 0.0,
            14: 0.0
        }

        # --- comando de apagado con postura final ---
        final_cmd = unitree_hg_msg_dds__LowCmd_()
        final_cmd.mode_pr = Mode.PR
        final_cmd.mode_machine = self.mode_machine_

        for i in range(G1_NUM_MOTOR):
            final_cmd.motor_cmd[i].mode = 1
            # si está en posiciones definidas, usarla; si no, dejar en 0.0
            final_cmd.motor_cmd[i].q = final_positions.get(i, 0.0)
            final_cmd.motor_cmd[i].dq = 0.0
            final_cmd.motor_cmd[i].tau = 0.0
            final_cmd.motor_cmd[i].kp = Kp[i]
            final_cmd.motor_cmd[i].kd = Kd[i]

        final_cmd.crc = self.crc.Crc(final_cmd)

        # enviar varias veces para asegurar que llegue
        for _ in range(repeat):
            self.lowcmd_publisher_.Write(final_cmd)
            time.sleep(delay)

        print("[INFO] Sent final shutdown posture to all joints.")
        
    def load_routine(self, filepath):
        if not os.path.isfile(filepath):
            raise FileNotFoundError(f"File {filepath} not found")

        ext = os.path.splitext(filepath)[1].lower()

        if ext == ".json":
            with open(filepath, "r") as f:
                routine = json.load(f)

        elif ext == ".txt":
            with open(filepath, "r") as f:
                content = f.read().strip()

            try:
                # Opción A: el TXT es en realidad JSON (como tu ejemplo)
                routine = json.loads(content)
            except json.JSONDecodeError:
                # Opción B: el TXT son pasos simples "joint value duration"
                routine = {"nombre_rutina": "txt_routine", "pasos": []}
                with open(filepath, "r") as f:
                    for line in f:
                        line = line.strip()
                        if not line:
                            continue
                        parts = line.split()
                        if len(parts) < 3:
                            continue
                        joint = int(parts[0])
                        value = float(parts[1])
                        duration = float(parts[2])
                        routine["pasos"].append({
                            "nombre": f"Paso {len(routine['pasos'])+1}",
                            "posiciones": {str(joint): value},
                            "duracion": duration
                        })

        else:
            raise ValueError("Unsupported file format. Use .json or .txt")

        return routine


    def execute_routine(self, routine):
        for step in routine["steps"]:
            positions = step["positions"]
            duration = step["duration"]
            self.move_to(positions, duration)

# ------------------ example usage ------------------
if __name__ == '__main__':
    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    # Channel init (igual que tu original)
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(1, "lo")

    custom = Custom(control_dt=0.002)
    custom.Init()

    # Esperar hasta que llegue el low_state (timeout a 5s)
    wait_t = 0.0
    timeout = 5.0
    while custom.low_state is None and wait_t < timeout:
        time.sleep(0.1)
        wait_t += 0.1

    if custom.low_state is None:
        print("[WARN] LowState not received within timeout; continuing but moves may be unsafe.")

    # Start writer thread
    custom.StartWriter()

    # ---------------- Opciones ----------------
    # Opción 1: rutina embebida
    # saludoR = {
    #   "nombre_rutina": "saludoR",
    #   "fecha_creacion": "2025-05-13 08:31:35",
    #   "numero_pasos": 11,
    #   "pasos": [
    #     {
    #       "nombre": "Paso 1",
    #       "posiciones": {
    #         "15": 0.2851693630218506,
    #         "16": 0.12492191791534424,
    #         "17": -0.009807109832763672,
    #         "18": 0.9769980907440186,
    #         "19": 0.07405257225036621,
    #         "20": 0.02068471908569336,
    #         "21": 0.036977291107177734,
    #         "22": 0.28391122817993164,
    #         "23": -0.12348365783691406,
    #         "24": 0.025542259216308594,
    #         "25": 0.982414722442627,
    #         "26": -0.16201698780059814,
    #         "27": 0.06299209594726562,
    #         "28": -0.018761634826660156,
    #         "12": -0.003699779510498047,
    #         "13": 0.0,
    #         "14": 0.0
    #       },
    #       "duracion": 1.0
    #     },
    #     {
    #       "nombre": "Paso 2",
    #       "posiciones": {
    #         "15": 0.20983648300170898,
    #         "16": 0.06774508953094482,
    #         "17": -0.009627342224121094,
    #         "18": 1.1561024188995361,
    #         "19": 0.07237482070922852,
    #         "20": 0.023154258728027344,
    #         "21": 0.0979914665222168,
    #         "22": -0.1253502368927002,
    #         "23": -1.9905662536621094,
    #         "24": -1.1774582862854004,
    #         "25": 0.27888059616088867,
    #         "26": 0.4338986277580261,
    #         "27": -0.06114530563354492,
    #         "28": -0.009994983673095703,
    #         "12": 0.13205385208129883,
    #         "13": 0.0,
    #         "14": 0.0
    #       },
    #       "duracion": 1.5
    #     }
    #   ]
    # }

    # Opción 2: cargar desde archivo externo
    # routine = custom.executor.load_routine("rutina.json")   # JSON
    routine = custom.load_routine("aplaudir.txt")    # TXT

    try:
        # Ejecuta opción 1:
        # custom.PlayRoutine(saludoR)

        # O si quieres usar archivo:
        custom.PlayRoutine(routine)

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C detectado. Interrumpiendo rutina...")
    finally:
        custom.StopAndShutdown()
        print("[INFO] Programa terminado.")
