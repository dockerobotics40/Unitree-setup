"""
# -----------------------------------------------------------------------------
# © 2025 Robotics 4.0.
# Este archivo forma parte de ejemplos y guías de uso distribuidos bajo
# la Licencia Apache 2.0.
#
# Puedes usarlo, modificarlo y redistribuirlo libremente citando la fuente.
# Nota: Este código es de carácter ilustrativo y no corresponde al producto
# completo de Robotics 4.0.
# -----------------------------------------------------------------------------
@file g1_armsdk_moveV4.py
@autor Sofía Milagros Castaño Vanegas - Robotics 4.0 Team
@date 2025-04-08
@version 1.0
@brief Control interactivo de articulaciones del G1 usando `arm_sdk` y guardado de datos para visualización.

Este script permite controlar manualmente las articulaciones del robot G1 de Unitree a través de la
interfaz `arm_sdk`, permitiendo movimientos suaves interpolados y secuencias personalizadas definidas
por el usuario. Emplea la API `loco_client` de la SDK2 de Unitree y facilita el control seguro del robot
en pruebas con sus brazos, cintura y otras articulaciones superiores.

@requisitos
- Conexión Ethernet activa entre el PC y el G1.
- Robot en modo normal de operación al inicio.
- SDK2 correctamente instalada y configurada.
- Versión G1 29DoF con articulaciones compatibles para brazos y torso.

@uso
    python3 g1_armsdk_moveV4.py <nombreInterfaz>

@funcionalidades
- Control de articulaciones superiores (brazos, cintura, muñecas) del G1.
- Interfaz interactiva para ingresar posiciones articulares en radianes de manera secuencial.
- Movimiento interpolado suave entre posiciones.
- Liberación progresiva del control para mayor seguridad.
"""
# Versión editada de g1_arm_sdk_moveV4.py con entrada solo manual y posición de descanso al liberar

import time
import sys
import numpy as np
import threading
import math
import csv
from datetime import datetime

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

class G1JointIndex:
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
    WaistYaw = 12
    WaistRoll = 13
    WaistPitch = 14
    kNotUsedJoint = 29

class Custom:
    def __init__(self):
        self.lock = threading.Lock()
        self.control_dt_ = 0.02
        self.kp = 60.
        self.kd = 1.5
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.first_update_low_state = False
        self.crc = CRC()
        self.stop_event = threading.Event()
        self.T = 5.0
        self.t = 0.0
        self.is_moving = False

        self.arm_joints = [
            G1JointIndex.LeftShoulderPitch, G1JointIndex.LeftShoulderRoll,
            G1JointIndex.LeftShoulderYaw, G1JointIndex.LeftElbow,
            G1JointIndex.LeftWristRoll, G1JointIndex.LeftWristPitch,
            G1JointIndex.LeftWristYaw,
            G1JointIndex.RightShoulderPitch, G1JointIndex.RightShoulderRoll,
            G1JointIndex.RightShoulderYaw, G1JointIndex.RightElbow,
            G1JointIndex.RightWristRoll, G1JointIndex.RightWristPitch,
            G1JointIndex.RightWristYaw,
            G1JointIndex.WaistYaw, G1JointIndex.WaistRoll, G1JointIndex.WaistPitch
        ]

        self.release_position = {
            G1JointIndex.LeftShoulderPitch: 0.294,
            G1JointIndex.LeftShoulderRoll: 0.227,
            G1JointIndex.LeftShoulderYaw: -0.0255,
            G1JointIndex.LeftElbow: 0.967,
            G1JointIndex.LeftWristRoll: 0.0794,
            G1JointIndex.LeftWristPitch: 0.0221,
            G1JointIndex.LeftWristYaw: 0.0030,
            G1JointIndex.RightShoulderPitch: 0.292,
            G1JointIndex.RightShoulderRoll: -0.225,
            G1JointIndex.RightShoulderYaw: 0.0308,
            G1JointIndex.RightElbow: 0.969,
            G1JointIndex.RightWristRoll: -0.147,
            G1JointIndex.RightWristPitch: 0.0286,
            G1JointIndex.RightWristYaw: 0.0085,
            G1JointIndex.WaistYaw: -0.0033,
            G1JointIndex.WaistRoll: 0.0,
            G1JointIndex.WaistPitch: 0.0
        }

        self.target_pos = {joint: 0.0 for joint in self.arm_joints}

        self.csv_file = open(f"data_g1_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", mode="w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        header = ["timestamp"]
        for joint in self.arm_joints:
            header.extend([f"q_joint{joint}", f"tau_joint{joint}"])
        self.csv_writer.writerow(header)
        self.sample_count = 0

    def Init(self):
        self.arm_sdk_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.arm_sdk_publisher.Init()
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    def Start(self):
        from unitree_sdk2py.utils.thread import RecurrentThread
        while not self.first_update_low_state:
            time.sleep(1)
        for joint in self.arm_joints:
            self.target_pos[joint] = self.low_state.motor_state[joint].q
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.LowCmdWrite, name="control")
        self.lowCmdWriteThreadPtr.Start()
        self.run_sequence()

    def LowStateHandler(self, msg: LowState_):
        with self.lock:
            self.low_state = msg
        if not self.first_update_low_state:
            self.first_update_low_state = True
        self.sample_count += 1
        if self.sample_count % 500 == 0:
            self.sample_count = 0
            row = [datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")]
            for joint in self.arm_joints:
                row.extend([msg.motor_state[joint].q, msg.motor_state[joint].tau_est])
            self.csv_writer.writerow(row)

    def interpolate_position(self, q_init, q_target):
        ratio = (1 - math.cos(math.pi * (self.t / self.T))) / 2 if self.t < self.T else 1.0
        return q_init + (q_target - q_init) * ratio

    def LowCmdWrite(self):
        if self.low_state is None:
            return
        with self.lock:
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1
            for joint in self.arm_joints:
                q_interp = self.interpolate_position(
                    self.low_state.motor_state[joint].q,
                    self.target_pos[joint])
                self.low_cmd.motor_cmd[joint].q = q_interp
                self.low_cmd.motor_cmd[joint].tau = 0.
                self.low_cmd.motor_cmd[joint].dq = 0.
                self.low_cmd.motor_cmd[joint].kp = self.kp
                self.low_cmd.motor_cmd[joint].kd = self.kd
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.arm_sdk_publisher.Write(self.low_cmd)
        self.t += self.control_dt_
        if self.t >= self.T:
            self.is_moving = False

    def move_to(self, target_positions, max_wait_time=6.0):
        self.target_pos = target_positions
        self.t = 0.0
        self.is_moving = True
        self.stop_event.clear()
        start_time = time.time()
        while self.is_moving:
            if time.time() - start_time > max_wait_time:
                print("Tiempo de espera excedido.")
                break
            if self.stop_event.is_set():
                print("Movimiento interrumpido.")
                break
            if self.has_reached_position(target_positions):
                print("Posición alcanzada.")
                break
            time.sleep(self.control_dt_)

    def has_reached_position(self, target_positions, tolerance=0.05):
        return all(abs(self.low_state.motor_state[j].q - target_positions[j]) <= tolerance
                   for j in self.arm_joints if j in target_positions)

    def release_control(self):
        print("\n➡️ Moviendo a posición de descanso...")
        self.move_to(self.release_position)
        for joint in self.arm_joints:
            self.low_cmd.motor_cmd[joint].q = self.low_state.motor_state[joint].q
            self.low_cmd.motor_cmd[joint].dq = 0.0
            self.low_cmd.motor_cmd[joint].tau = 0.0
            self.low_cmd.motor_cmd[joint].kp = 0.0
            self.low_cmd.motor_cmd[joint].kd = 0.0
        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 0
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.arm_sdk_publisher.Write(self.low_cmd)
        self.csv_file.close()
        print("📁 CSV cerrado. Control liberado.")

    def get_user_joint_positions(self):
        pos = {}
        for joint in self.arm_joints:
            name = next((k for k, v in G1JointIndex.__dict__.items() if v == joint), str(joint))
            val = input(f"{name} (rad, enter = 0): ")
            if val.lower() == 'exit':
                return None
            try:
                pos[joint] = float(val) if val.strip() else 0.0
            except:
                print("❌ Entrada inválida.")
                return None
        return pos

    def run_sequence(self):
        input("Presiona Enter para mover a cero...")
        self.move_to({joint: 0.0 for joint in self.arm_joints})
        while True:
            print("\nMenú:")
            print("1. Ingresar nueva posición")
            print("2. Volver a cero")
            opt = input("Opción (1/2): ")
            if opt == '1':
                pos = self.get_user_joint_positions()
                if pos:
                    self.move_to(pos)
            elif opt == '2':
                self.move_to({joint: 0.0 for joint in self.arm_joints})
                print("\n1. Nueva posición\n2. Salir y liberar control")
                sub = input("Opción (1/2): ")
                if sub == '1':
                    continue
                elif sub == '2':
                    input("Presiona Enter para liberar...")
                    self.release_control()
                    sys.exit(0)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(f"Uso: python3 {sys.argv[0]} <interfaz>")
        sys.exit(-1)
    print("⚠️ Asegúrate de que no haya obstáculos cerca del robot.")
    input("Presiona Enter para continuar...")
    ChannelFactoryInitialize(0, sys.argv[1])
    custom = Custom()
    custom.Init()
    custom.Start()
