#!/usr/bin/env python3
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
@file g1_armsdk_moveV4_23dof.py
@autor Sofía Milagros Castaño Vanegas - Robotics 4.0 Team
@date 2025-04-08
@version 1.1
@brief Control interactivo de articulaciones superiores del G1 EDU de 23 DoF mediante arm_sdk.

Este script conserva la lógica funcional de g1_armsdk_moveV4.py para un
Unitree G1 EDU de 23 DoF:

1. Se conecta al robot mediante la interfaz de red indicada.
2. Publica comandos articulares en el tópico `rt/arm_sdk`.
3. Lee posiciones y torques desde `rt/lowstate`.
4. Inicializa los objetivos con la posición articular actual.
5. Mueve suavemente las articulaciones superiores hacia cero.
6. Permite introducir posiciones articulares manualmente en radianes.
7. Interpola cada transición mediante una trayectoria cosenoidal de 5 segundos.
8. Registra posición y torque estimado en un archivo CSV.
9. Antes de salir, regresa a una posición de descanso y libera `arm_sdk`.

La versión de 23 DoF controla únicamente las articulaciones físicamente
disponibles en esta configuración:

- Cintura: WaistYaw.
- Brazo izquierdo: ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow y WristRoll.
- Brazo derecho: ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow y WristRoll.

No se envían comandos a WaistRoll, WaistPitch, WristPitch ni WristYaw porque
esas articulaciones no existen en el G1 de 23 DoF.

@requisitos
- Conexión Ethernet activa entre el PC y el G1.
- Robot encendido, estable y en modo normal de operación.
- Unitree SDK2 para Python instalada y configurada.
- Servicio de locomoción compatible con el tópico `rt/arm_sdk`.
- Configuración física G1 EDU de 23 DoF.
- Área segura y libre de obstáculos alrededor de brazos y torso.

@uso
    python3 g1_armsdk_moveV4_23dof.py <nombreInterfaz>

@funcionalidades
- Control manual de brazos, WristRoll y WaistYaw.
- Entrada secuencial de posiciones articulares en radianes.
- Movimiento interpolado suave.
- Retorno a cero.
- Posición de descanso antes de liberar el control.
- Registro CSV de posición y torque estimado.
"""

import csv
import math
import sys
import threading
import time
from datetime import datetime

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import (
    unitree_hg_msg_dds__LowCmd_,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread


class G1JointIndex:
    """
    Índices de motor relevantes para el G1.

    La estructura DDS conserva la numeración general del G1 de 29 motores,
    aunque algunos índices no correspondan a motores físicos en la versión
    de 23 DoF.
    """

    WaistYaw = 12

    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19

    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26

    # Índice reservado utilizado por arm_sdk:
    # q = 1 habilita arm_sdk y q = 0 lo libera.
    kNotUsedJoint = 29


class Custom:
    def __init__(self):
        self.lock = threading.Lock()

        # Periodo y ganancias conservados respecto al código de 29 DoF.
        self.control_dt_ = 0.02
        self.kp = 60.0
        self.kd = 1.5

        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.first_update_low_state = False
        self.crc = CRC()

        self.stop_event = threading.Event()

        # Duración de la interpolación cosenoidal.
        self.T = 5.0
        self.t = 0.0
        self.is_moving = False

        # Evita que el hilo periódico vuelva a publicar y reactive arm_sdk
        # después de la liberación final.
        self.control_enabled = True
        self.control_released = False

        # Articulaciones superiores disponibles en el G1 de 23 DoF.
        self.arm_joints = [
            G1JointIndex.LeftShoulderPitch,
            G1JointIndex.LeftShoulderRoll,
            G1JointIndex.LeftShoulderYaw,
            G1JointIndex.LeftElbow,
            G1JointIndex.LeftWristRoll,
            G1JointIndex.RightShoulderPitch,
            G1JointIndex.RightShoulderRoll,
            G1JointIndex.RightShoulderYaw,
            G1JointIndex.RightElbow,
            G1JointIndex.RightWristRoll,
            G1JointIndex.WaistYaw,
        ]

        # Mismos valores del código original para las articulaciones que
        # sí existen en el G1 de 23 DoF.
        self.release_position = {
            G1JointIndex.LeftShoulderPitch: 0.294,
            G1JointIndex.LeftShoulderRoll: 0.227,
            G1JointIndex.LeftShoulderYaw: -0.0255,
            G1JointIndex.LeftElbow: 0.967,
            G1JointIndex.LeftWristRoll: 0.0794,
            G1JointIndex.RightShoulderPitch: 0.292,
            G1JointIndex.RightShoulderRoll: -0.225,
            G1JointIndex.RightShoulderYaw: 0.0308,
            G1JointIndex.RightElbow: 0.969,
            G1JointIndex.RightWristRoll: -0.147,
            G1JointIndex.WaistYaw: -0.0033,
        }

        self.target_pos = {joint: 0.0 for joint in self.arm_joints}

        self.joint_names = {
            value: name
            for name, value in vars(G1JointIndex).items()
            if not name.startswith("_") and isinstance(value, int)
        }

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file = open(
            f"data_g1_23dof_{timestamp}.csv",
            mode="w",
            newline="",
            encoding="utf-8",
        )
        self.csv_writer = csv.writer(self.csv_file)

        header = ["timestamp"]
        for joint in self.arm_joints:
            header.extend([f"q_joint{joint}", f"tau_joint{joint}"])
        self.csv_writer.writerow(header)

        self.sample_count = 0
        self.lowCmdWriteThreadPtr = None

    def Init(self):
        """Inicializa el publicador de arm_sdk y el suscriptor de lowstate."""
        self.arm_sdk_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.arm_sdk_publisher.Init()

        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    def Start(self):
        """
        Espera el primer estado válido, captura la postura actual como objetivo,
        inicia el hilo de control y abre la secuencia interactiva.
        """
        print("Esperando el primer mensaje de rt/lowstate...")
        while not self.first_update_low_state:
            time.sleep(1.0)

        with self.lock:
            for joint in self.arm_joints:
                self.target_pos[joint] = self.low_state.motor_state[joint].q

        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_,
            target=self.LowCmdWrite,
            name="control",
        )
        self.lowCmdWriteThreadPtr.Start()

        self.run_sequence()

    def LowStateHandler(self, msg: LowState_):
        """
        Actualiza el último estado recibido y registra periódicamente posición
        y torque estimado de las articulaciones controladas.
        """
        with self.lock:
            self.low_state = msg

            if not self.first_update_low_state:
                self.first_update_low_state = True

            self.sample_count += 1
            if self.sample_count % 500 == 0:
                self.sample_count = 0

                row = [datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")]
                for joint in self.arm_joints:
                    row.extend(
                        [
                            msg.motor_state[joint].q,
                            msg.motor_state[joint].tau_est,
                        ]
                    )

                self.csv_writer.writerow(row)
                self.csv_file.flush()

    def interpolate_position(self, q_init, q_target):
        """
        Calcula una interpolación cosenoidal entre la posición inicial y la
        posición objetivo, conservando la ecuación del código original.
        """
        if self.t < self.T:
            ratio = (1.0 - math.cos(math.pi * (self.t / self.T))) / 2.0
        else:
            ratio = 1.0

        return q_init + (q_target - q_init) * ratio

    def LowCmdWrite(self):
        """
        Publica a 50 Hz los objetivos articulares interpolados mediante
        rt/arm_sdk.
        """
        if not self.control_enabled:
            return

        with self.lock:
            if not self.control_enabled or self.low_state is None:
                return

            # 1: habilitar arm_sdk.
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0

            for joint in self.arm_joints:
                q_interp = self.interpolate_position(
                    self.low_state.motor_state[joint].q,
                    self.target_pos[joint],
                )

                self.low_cmd.motor_cmd[joint].q = q_interp
                self.low_cmd.motor_cmd[joint].tau = 0.0
                self.low_cmd.motor_cmd[joint].dq = 0.0
                self.low_cmd.motor_cmd[joint].kp = self.kp
                self.low_cmd.motor_cmd[joint].kd = self.kd

            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.arm_sdk_publisher.Write(self.low_cmd)

            self.t += self.control_dt_
            if self.t >= self.T:
                self.is_moving = False

    def move_to(self, target_positions, max_wait_time=6.0):
        """
        Inicia una transición hacia un diccionario de posiciones objetivo y
        espera hasta que finalice, se alcance la tolerancia o venza el tiempo.
        """
        missing_joints = [
            joint for joint in self.arm_joints if joint not in target_positions
        ]
        if missing_joints:
            raise ValueError(
                "La posición objetivo no contiene todas las articulaciones "
                f"controladas: {missing_joints}"
            )

        with self.lock:
            self.target_pos = dict(target_positions)
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

        self.is_moving = False

    def has_reached_position(self, target_positions, tolerance=0.05):
        """Comprueba si todas las articulaciones están dentro de la tolerancia."""
        with self.lock:
            if self.low_state is None:
                return False

            return all(
                abs(
                    self.low_state.motor_state[joint].q
                    - target_positions[joint]
                )
                <= tolerance
                for joint in self.arm_joints
            )

    def release_control(self, move_to_rest=True):
        """
        Opcionalmente lleva el robot a la posición de descanso y después
        libera arm_sdk sin permitir que el hilo periódico vuelva a habilitarlo.
        """
        if self.control_released:
            return

        if move_to_rest and self.low_state is not None:
            print("\n➡️ Moviendo a posición de descanso...")
            self.move_to(self.release_position)

        with self.lock:
            # Detiene nuevas publicaciones del hilo antes de enviar q = 0.
            self.control_enabled = False
            self.is_moving = False
            self.stop_event.set()

            if self.low_state is not None:
                for joint in self.arm_joints:
                    self.low_cmd.motor_cmd[joint].q = (
                        self.low_state.motor_state[joint].q
                    )
                    self.low_cmd.motor_cmd[joint].dq = 0.0
                    self.low_cmd.motor_cmd[joint].tau = 0.0
                    self.low_cmd.motor_cmd[joint].kp = 0.0
                    self.low_cmd.motor_cmd[joint].kd = 0.0

            # 0: liberar arm_sdk.
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 0.0
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.arm_sdk_publisher.Write(self.low_cmd)

            self.control_released = True

        if not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()

        print("📁 CSV cerrado. Control liberado.")

    def get_user_joint_positions(self):
        """
        Solicita una posición en radianes para cada articulación disponible.
        Enter conserva la lógica original y asigna 0 rad.
        """
        positions = {}

        for joint in self.arm_joints:
            name = self.joint_names.get(joint, str(joint))
            value = input(f"{name} (rad, Enter = 0, exit = cancelar): ")

            if value.strip().lower() == "exit":
                return None

            try:
                positions[joint] = (
                    float(value) if value.strip() else 0.0
                )
            except ValueError:
                print("❌ Entrada inválida.")
                return None

        return positions

    def run_sequence(self):
        """
        Conserva el flujo interactivo del código de 29 DoF:
        cero -> nueva posición o cero -> nueva posición o salida.
        """
        input("Presiona Enter para mover a cero...")
        self.move_to({joint: 0.0 for joint in self.arm_joints})

        while True:
            print("\nMenú:")
            print("1. Ingresar nueva posición")
            print("2. Volver a cero")

            option = input("Opción (1/2): ").strip()

            if option == "1":
                positions = self.get_user_joint_positions()
                if positions is not None:
                    self.move_to(positions)

            elif option == "2":
                self.move_to({joint: 0.0 for joint in self.arm_joints})

                print("\n1. Nueva posición")
                print("2. Salir y liberar control")
                suboption = input("Opción (1/2): ").strip()

                if suboption == "1":
                    continue

                if suboption == "2":
                    input("Presiona Enter para liberar...")
                    self.release_control(move_to_rest=True)
                    return

                print("❌ Opción inválida.")

            else:
                print("❌ Opción inválida.")


def main():
    if len(sys.argv) != 2:
        print(f"Uso: python3 {sys.argv[0]} <interfaz>")
        sys.exit(-1)

    print("⚠️ Asegúrate de que no haya obstáculos cerca del robot.")
    print("⚠️ Mantén acceso inmediato al control remoto y al paro de emergencia.")
    input("Presiona Enter para continuar...")

    ChannelFactoryInitialize(0, sys.argv[1])

    custom = Custom()
    custom.Init()

    try:
        custom.Start()

    except KeyboardInterrupt:
        print("\nEjecución interrumpida por el usuario.")
        # En una interrupción no se fuerza un movimiento adicional:
        # se libera inmediatamente el control articular.
        custom.release_control(move_to_rest=False)

    except Exception as error:
        print(f"\nError durante la ejecución: {error}")
        custom.release_control(move_to_rest=False)
        raise

    finally:
        if not custom.control_released:
            custom.release_control(move_to_rest=False)


if __name__ == "__main__":
    main()
