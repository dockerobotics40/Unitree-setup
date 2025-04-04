"""
@file g1_armsdk_moveV2.py
@autor Sofía Milagros Castaño Vanegas - Robotics 4.0 Team
@date 2025-03-22
@version 2.0
@brief Control interactivo de articulaciones del G1 usando `arm_sdk`.

Este script permite controlar manualmente las articulaciones del robot G1 de Unitree a través de la
interfaz `arm_sdk`, permitiendo movimientos suaves interpolados y secuencias personalizadas definidas
por el usuario. Emplea la API `loco_client` de la SDK2 de Unitree y facilita el control seguro del robot
en pruebas con sus brazos, cintura y otras articulaciones superiores.

@requisitos
- Conexión Ethernet activa entre el PC y el G1.
- Robot en modo normal (cero torque) al inicio.
- SDK2 correctamente instalada y configurada.
- Versión G1 29DoF para articulaciones compatibles con brazos y torso.

@uso
    python3 g1_armsdk_moveV2.py <nombreInterfaz>

@funcionalidades
- Control de articulaciones superiores (brazos, cintura, muñecas) del G1.
- Interfaz interactiva para ingresar posiciones articulares en radianes.
- Movimiento interpolado suave entre posiciones.
- Liberación progresiva del control para mayor seguridad.
"""
import time
import sys
import numpy as np
import threading
import math

# Importación de módulos de la SDK de Unitree para comunicación y control
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

class G1JointIndex:
    """
    Índices de las articulaciones del robot G1 de Unitree.
    Se incluyen piernas, brazos y cintura.
    """
    # Pierna izquierda
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleB = 4
    LeftAnkleRoll = 5
    LeftAnkleA = 5

    # Pierna derecha
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleB = 10
    RightAnkleRoll = 11
    RightAnkleA = 11

    # Cintura
    WaistYaw = 12
    WaistRoll = 13        # No válido para G1 23DoF/29DoF con cintura bloqueada
    WaistA = 13           # No válido para G1 23DoF/29DoF con cintura bloqueada
    WaistPitch = 14       # No válido para G1 23DoF/29DoF con cintura bloqueada
    WaistB = 14           # No válido para G1 23DoF/29DoF con cintura bloqueada

    # Brazo izquierdo
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20   # No válido para G1 23DoF
    LeftWristYaw = 21     # No válido para G1 23DoF

    # Brazo derecho
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27  # No válido para G1 23DoF
    RightWristYaw = 28    # No válido para G1 23DoF

    kNotUsedJoint = 29  # Articulación no utilizada (peso)
    
class Custom:
    """Clase para controlar los movimientos del robot G1 de Unitree."""
    def __init__(self):
        self.lock = threading.Lock()  # Bloqueo para sincronización
        self.control_dt_ = 0.02  # Intervalo de control (20 ms)
        self.kp = 60.  # Ganancia proporcional
        self.kd = 1.5  # Ganancia derivativa
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()  
        self.low_state = None  
        self.first_update_low_state = False
        self.crc = CRC()
        self.done = False  
        self.current_stage = 0  
        self.T = 5.0  # Duración del movimiento
        self.t = 0.0  
        self.is_moving = False  # Inicialización corregida
        self.stop_event = threading.Event()  # Para evitar bloqueos indefinidos en move_to()

        # Lista de articulaciones controladas
        self.arm_joints = [
            G1JointIndex.LeftShoulderPitch,  G1JointIndex.LeftShoulderRoll,
            G1JointIndex.LeftShoulderYaw,    G1JointIndex.LeftElbow,
            G1JointIndex.LeftWristRoll,      G1JointIndex.LeftWristPitch,
            G1JointIndex.LeftWristYaw,
            G1JointIndex.RightShoulderPitch, G1JointIndex.RightShoulderRoll,
            G1JointIndex.RightShoulderYaw,   G1JointIndex.RightElbow,
            G1JointIndex.RightWristRoll,     G1JointIndex.RightWristPitch,
            G1JointIndex.RightWristYaw,
            G1JointIndex.WaistYaw,
            G1JointIndex.WaistRoll,
            G1JointIndex.WaistPitch
        ]

        self.target_pos = {joint: 0.0 for joint in self.arm_joints}  
        self.alpha = 0.05  

    def Init(self):
        """Inicializa la comunicación con el robot."""
        # Publicador para enviar comandos al robot
        self.arm_sdk_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.arm_sdk_publisher.Init()

        # Suscriptor para recibir el estado del robot
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)
    def Start(self):
        """Espera el primer estado del robot y comienza el control."""
        try:
            self.lowCmdWriteThreadPtr = RecurrentThread(
                interval=self.control_dt_, target=self.LowCmdWrite, name="control"
            ) 

            while not self.first_update_low_state:
                time.sleep(1)

            if self.first_update_low_state: 
                for joint in self.arm_joints:
                    self.target_pos[joint] = self.low_state.motor_state[joint].q#Se toma posicion inicial
                self.lowCmdWriteThreadPtr.Start()
                self.run_sequence()
        except KeyboardInterrupt:
            print("\nInterrupción detectada. Liberando el control...")
            self.release_control()
            return


    def LowStateHandler(self, msg: LowState_):
        """Recibe y actualiza el estado del robot."""
        with self.lock:  # Protección contra concurrencia
            self.low_state = msg

        if not self.first_update_low_state:
            self.first_update_low_state = True

    def interpolate_position(self, q_init, q_target):
        """Interpolación de posición para un movimiento suave."""
        ratio = (1 - math.cos(math.pi * (self.t / self.T))) / 2 if self.t < self.T else 1.0
        return q_init + (q_target - q_init) * ratio

    def LowCmdWrite(self):
        """Interpolación sinusoidal y envío de comandos al robot."""
        if self.low_state is None:
            return  # No enviar comandos si no hay estado disponible

        with self.lock:  
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q =  1 # 1:Enable arm_sdk, 0:Disable arm_sdk
            for  joint in self.arm_joints:
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
                self.is_moving = False  # Asegurar que se detenga la interpolación

    def move_to(self, target_positions, max_wait_time=6.0):
        """Mueve el robot a la posición objetivo asegurando que la interpolación se complete."""
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
        """Verifica si el robot ha alcanzado la posición objetivo."""
        if self.low_state is None:
            return False

        for joint in self.arm_joints:
            if joint not in target_positions:
                continue  # Evita errores si falta alguna articulación
            if abs(self.low_state.motor_state[joint].q - target_positions[joint]) > tolerance:
                return False
        return True

    def release_control(self):
        """Libera el control de manera progresiva."""
        self.stop_event.set()  # Detener posibles movimientos bloqueantes
        
        duration = 2.0  # Duración de la transición en segundos
        steps = int(duration / self.control_dt_)  # Cantidad de pasos según dt
        
        for step in range(steps):
            with self.lock:
                ratio = (step + 1) / steps  # Interpolación lineal de 0 a 1
                self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1 - ratio  # 1:Enable arm_sdk, 0:Disable arm_sdk

                self.low_cmd.crc = self.crc.Crc(self.low_cmd)  # Actualizar CRC
                self.arm_sdk_publisher.Write(self.low_cmd)  # Enviar comando

                time.sleep(self.control_dt_)  # Esperar 20ms por cada iteración  


        print("\nControl liberado completamente.")
        
    def get_user_joint_positions(self):
        """Permite al usuario configurar las posiciones deseadas de las articulaciones."""
        target_pos = {joint: 0.0 for joint in self.arm_joints}
        print("\nConfiguración de posiciones articulares:")

        for joint in self.arm_joints:
            joint_name = next((name for name, value in G1JointIndex.__dict__.items() if value == joint), None)
            if joint_name is None:
                continue  # Evita nombres inválidos

            while True:
                user_input = input(f"Ingrese posición para {joint_name} (rad) o deje vacío para 0 (Escriba 'exit' para salir): ")
                
                if user_input.lower() == "exit":
                    print("\nCancelando configuración y volviendo al menú principal.")
                    self.release_control()
                    return None  # Indica que el usuario quiere salir
                
                if user_input.strip() == "":
                    target_pos[joint] = 0.0
                    break  # Continúa con la siguiente articulación
                
                try:
                    pos = float(user_input)
                    target_pos[joint] = pos
                    break  # Continúa con la siguiente articulación
                except ValueError:
                    print("Entrada no válida. Intente de nuevo.")

        return target_pos

    def run_sequence(self):
        """Ejecuta la secuencia de movimientos con confirmaciones del usuario."""
        input("\nMoviendo a posición cero... Presione Enter para continuar.")
        self.move_to({joint: 0.0 for joint in self.arm_joints})

        input("\nConfigurar posición objetivo... Presione Enter para continuar.")
        target_positions = self.get_user_joint_positions()
        if target_positions is None:
            print("\nCancelando la secuencia.")
            self.release_control()
            sys.exit(0)

        self.move_to(target_positions)

        input("\nVolviendo a posición cero... Presione Enter para continuar.")
        self.move_to({joint: 0.0 for joint in self.arm_joints})

        input("\nLiberando el control... Presione Enter para continuar.")
        self.release_control()
        
        print("\nSecuencia finalizada. Saliendo del programa...")
        sys.exit(0)  # Salida limpia después de liberar el control


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(f"Uso: python3 {sys.argv[0]} networkInterface")
        sys.exit(-1)

    print("ADVERTENCIA: Asegúrese de que no haya obstáculos cerca del robot.")
    input("Presione Enter para continuar...")

    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    custom = Custom()
    custom.Init()
    custom.Start()
