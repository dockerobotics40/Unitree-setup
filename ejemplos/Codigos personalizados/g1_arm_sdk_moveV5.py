"""
    @file g1_armsdk_moveV5.py
    @author Sofía Milagros Castaño Vanegas - Robotics 4.0 Team
    @date 2025-04-08
    @version 1.1
    @brief Control interactivo y por archivo de las articulaciones del G1 usando `arm_sdk`, con movimientos interpolados y registro de datos.

    Este script permite controlar las articulaciones superiores del robot G1 de Unitree a través de la interfaz `arm_sdk`,
    utilizando la API `loco_client` de la SDK2 para definición de trayectorias de caminata. Su modo de operación es con ingreso manual de las posiciones.
    Se incluye interpolación suave entre posiciones y almacenamiento automático de datos
    para análisis o visualización posterior.

    @requisitos
    - Conexión Ethernet activa entre el PC y el G1.
    - Robot en modo normal de operación al inicio.
    - SDK2 correctamente instalada y configurada.
    - Versión G1 29DoF con articulaciones compatibles para brazos, muñecas y torso.

    @uso
        python3 g1_armsdk_moveV5.py <nombreInterfaz> 

        - <nombreInterfaz>: nombre de la interfaz de red conectada al robot (ej. 'lo', 'eth0').

    @funcionalidades
    - Control de articulaciones superiores (brazos, cintura, muñecas) del G1.
    - Modo manual: ingreso interactivo de posiciones articulares en consola (en radianes).
    - Modo por archivo: lectura de secuencias desde archivo `.txt`, línea por línea.
    - Movimiento interpolado suave entre posiciones para mayor seguridad y fluidez.
    - Verificación de posición alcanzada al final de cada movimiento.
    - Registro automático de posiciones y torques en archivo `.csv` con timestamp.
    - Liberación progresiva del control y finalización segura del script.
"""

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
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

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
        self.done = False
        self.current_stage = 0
        self.T = 5.0
        self.t = 0.0
        self.is_moving = False
        self.stop_event = threading.Event()
        self.DEFAULT_X_VEL = 0.3
        self.DEFAULT_Y_VEL = 0.3
        self.DEFAULT_YAW_VEL = 0.5

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
        self.alpha = 0.05

        self.csv_file = open(f"data_g1_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", mode="w", newline="")
        self.csv_writer = csv.writer(self.csv_file)

        header = ["timestamp"]
        for joint in self.arm_joints:
            header.extend([f"q_joint{joint}", f"tau_joint{joint}"])
        self.csv_writer.writerow(header)

        self.sample_count = 0

        self.client = LocoClient()
        self.client.SetTimeout(10.0)
        self.client.Init()

    def Init(self):
        self.arm_sdk_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.arm_sdk_publisher.Init()

        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    def Start(self):
        try:
            self.lowCmdWriteThreadPtr = RecurrentThread(
                interval=self.control_dt_, target=self.LowCmdWrite, name="control")

            while not self.first_update_low_state:
                time.sleep(1)

            if self.first_update_low_state:
                for joint in self.arm_joints:
                    self.target_pos[joint] = self.low_state.motor_state[joint].q
                self.lowCmdWriteThreadPtr.Start()
                self.run_sequence()
        except KeyboardInterrupt:
            print("\nInterrupción detectada. Liberando el control...")
            self.release_control()
            return

    def LowStateHandler(self, msg: LowState_):
        with self.lock:
            self.low_state = msg

        if not self.first_update_low_state:
            self.first_update_low_state = True

        self.sample_count += 1
        if self.sample_count % 500 == 0:
            self.sample_count = 0
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
            row = [timestamp]
            for joint in self.arm_joints:
                tau = msg.motor_state[joint].tau_est
                q = msg.motor_state[joint].q
                row.extend([q, tau])
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
                q_init = self.low_state.motor_state[joint].q
                q_target = self.target_pos[joint]
                q_interpolated = self.interpolate_position(q_init, q_target)

                self.low_cmd.motor_cmd[joint].q = q_interpolated
                self.low_cmd.motor_cmd[joint].tau = 0.
                self.low_cmd.motor_cmd[joint].dq = 0.
                self.low_cmd.motor_cmd[joint].kp = self.kp
                self.low_cmd.motor_cmd[joint].kd = self.kd

            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.arm_sdk_publisher.Write(self.low_cmd)

        with self.lock:
            self.t += self.control_dt_
            if self.t >= self.T:
                self.is_moving = False

    def move_to(self, target_positions, max_wait_time=6.0):
        with self.lock:
            self.target_pos = target_positions
            self.t = 0.0
            self.is_moving = True
            self.stop_event.clear()

        start_time = time.time()

        while True:
            with self.lock:
                if not self.is_moving:
                    print("Movimiento completado.")
                    break

            if time.time() - start_time > max_wait_time:
                print("Advertencia: Tiempo de espera excedido. Posición no alcanzada.")
                break

            if self.stop_event.is_set():
                print("Movimiento detenido por solicitud.")
                break

            if self.has_reached_position(target_positions, tolerance=0.05):
                print("Posición alcanzada con éxito.")
                break

            time.sleep(self.control_dt_)

    def has_reached_position(self, target_positions, tolerance=0.05):
        if self.low_state is None:
            return False

        for joint in self.arm_joints:
            if joint not in target_positions:
                continue
            if abs(self.low_state.motor_state[joint].q - target_positions[joint]) > tolerance:
                return False
        return True

    def release_control(self):
        self.stop_event.set()
        print("\nMoviendo a posición de descanso antes de liberar control...")
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
        print("Archivo CSV cerrado.\nControl liberado completamente.")

    def get_user_joint_positions(self):
        target_pos = {joint: 0.0 for joint in self.arm_joints}
        print("\nConfiguración de posiciones articulares:")
        for joint in self.arm_joints:
            joint_name = next((name for name, value in G1JointIndex.__dict__.items() if value == joint), None)
            if joint_name is None:
                continue
            while True:
                user_input = input(f"Ingrese posición para {joint_name} (rad) o deje vacío para 0 (Escriba 'exit' para salir): ")
                if user_input.lower() == "exit":
                    print("\nCancelando configuración y volviendo al menú principal.")
                    self.release_control()
                    return None
                if user_input.strip() == "":
                    target_pos[joint] = 0.0
                    break
                try:
                    pos = float(user_input)
                    target_pos[joint] = pos
                    break
                except ValueError:
                    print("Entrada no válida. Intente de nuevo.")
        return target_pos

    def get_walk_trajectory_from_user(self):
        direction_map = {
            "adelante":  (self.DEFAULT_X_VEL, 0.0, 0.0),
            "atras":     (-self.DEFAULT_X_VEL, 0.0, 0.0),
            "izquierda": (0.0, self.DEFAULT_Y_VEL, 0.0),
            "derecha":   (0.0, -self.DEFAULT_Y_VEL, 0.0),
            "rotar_izq":  (0.0, 0.0,  self.DEFAULT_YAW_VEL),
            "rotar_der":  (0.0, 0.0, -self.DEFAULT_YAW_VEL),
        }
        trajectory = []
        print("\nIngresa los pasos de la trayectoria. Escribe 'fin' para terminar.")
        while True:
            direction = input("Dirección (adelante, atrás, izquierda, derecha, rotar_izq, rotar_der, fin): ").strip().lower()
            if direction == "fin":
                break
            if direction not in direction_map:
                print(" Dirección no válida. Intenta otra vez.")
                continue
            try:
                duration = float(input("Duración (s): "))
                x, y, yaw = direction_map[direction]
                trajectory.append((x, y, yaw, duration))
            except ValueError:
                print("Duración inválida. Intenta otra vez.")
        return trajectory

    def execute_trajectory_sequence(self, movimientos):
        if not movimientos:
            print(" La secuencia está vacía. No se ejecutará ningún movimiento.")
            return
        print("\n Secuencia definida:")
        for i, (x, y, yaw, duration) in enumerate(movimientos, 1):
            print(f"  #{i}: x={x}, y={y}, yaw={yaw}, duración={duration}s")
        confirmar = input("\n¿Deseas ejecutar esta secuencia? (s/n): ").strip().lower()
        if confirmar != 's':
            print(" Ejecución cancelada.")
            return
        print("\n Ejecutando secuencia...")
        for i, (x, y, yaw, duration) in enumerate(movimientos, 1):
            print(f"\n Movimiento #{i}")
            self.move(self.client, x_vel=x, y_vel=y, yaw_vel=yaw, duration=duration)
        print("\n Secuencia de movimientos completada.")

    def move(self, client, x_vel=0.0, y_vel=0.0, yaw_vel=0.0, duration=2.0):
        print(f"\n➡️ Iniciando movimiento: x_vel={x_vel}, y_vel={y_vel}, yaw_vel={yaw_vel}, duración={duration}s")
        self.client.Move(x_vel, y_vel, yaw_vel, True)
        time.sleep(duration)
        self.client.Move(0, 0, 0)
        print("Movimiento finalizado.\n")
        time.sleep(0.8)

    def ask_walk_after_arm_motion(self):
        choice = input("\n¿Deseas que el robot ejecute una trayectoria ahora? (s/n): ").strip().lower()
        if choice == 's':
            trajectory = self.get_walk_trajectory_from_user()
            self.execute_trajectory_sequence(trajectory)
        else:
            print(" Caminata omitida.")

    def run_sequence(self):
        input("\nMoviendo a posición cero... Presione Enter para continuar.")
        self.move_to({joint: 0.0 for joint in self.arm_joints})
        self.ask_walk_after_arm_motion()
        while True:
            print("\nOpciones:")
            print("1. Ingresar nueva posición objetivo")
            print("2. Volver a posición cero")
            option = input("Seleccione una opción (1/2): ").strip()
            if option == "1":
                target_positions = self.get_user_joint_positions()
                if target_positions is None:
                    print("\nCancelando la secuencia.")
                    continue
                self.move_to(target_positions)
                self.ask_walk_after_arm_motion()
            elif option == "2":
                self.move_to({joint: 0.0 for joint in self.arm_joints})
                self.ask_walk_after_arm_motion()
                print("\nOpciones tras volver a cero:")
                print("1. Ingresar otra posición objetivo")
                print("2. Liberar control y salir")
                sub_option = input("Seleccione una opción (1/2): ").strip()
                if sub_option == "1":
                    continue
                elif sub_option == "2":
                    input("\nLiberando el control... Presione Enter para continuar.")
                    self.release_control()
                    print("\nControl liberado. Saliendo del programa...")
                    sys.exit(0)
                else:
                    print("Opción inválida. Regresando al menú principal.")
            else:
                print("Opción inválida. Intente nuevamente.")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(f"Uso: python3 {sys.argv[0]} networkInterface")
        sys.exit(-1)
    print("ADVERTENCIA: Asegúrese de que no haya obstáculos cerca del robot.")
    input("Presione Enter para continuar...")
    ChannelFactoryInitialize(0, sys.argv[1])
    custom = Custom()
    custom.Init()
    custom.Start()
