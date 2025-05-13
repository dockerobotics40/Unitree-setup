"""
    @file g1_armsdk_moveV5.py
    @author Sofía Milagros Castaño Vanegas - Robotics 4.0 Team
    @date 2025-04-08
    @version 1.1
    @brief Control interactivo y por archivo de las articulaciones del G1 usando `arm_sdk`, con movimientos interpolados y registro de datos.

    Este script permite controlar las articulaciones superiores del robot G1 de Unitree a través de la interfaz `arm_sdk`,
    utilizando la API `loco_client` de la SDK2 para definición de trayectorias de caminata. Soporta dos modos de operación: ingreso manual por consola o ejecución de
    secuencias desde un archivo `.txt`. Se incluye interpolación suave entre posiciones y almacenamiento automático de datos
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

# Importación de módulos de la SDK de Unitree para comunicación y control
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
        self.DEFAULT_X_VEL = 0.3  # m/s hacia adelante/atrás
        self.DEFAULT_Y_VEL = 0.3  # m/s lateral
        self.DEFAULT_YAW_VEL = 0.5  # rad/s para giro

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
        
        #Creación del CSV
        self.csv_file = open(f"data_g1_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", mode="w", newline="")
        self.csv_writer = csv.writer(self.csv_file)

        # Escribe encabezado del CSV
                
        header = ["timestamp"]
        
        for joint in self.arm_joints:
            header.extend([f"q_joint{joint}", f"tau_joint{joint}"])
        self.csv_writer.writerow(header)

        
        self.sample_count = 0

        self.client = LocoClient()
        self.client.SetTimeout(10.0)  # Establece el tiempo máximo de espera de respuesta
        self.client.Init()

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
            
        self.sample_count += 1
        if self.sample_count % 500 == 0:
            self.sample_count = 0
            # Guardar los valores de torque en el CSV
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
            row = [timestamp]

            for joint in self.arm_joints:
                tau = msg.motor_state[joint].tau_est
                q = msg.motor_state[joint].q
                row.extend([q, tau])

            self.csv_writer.writerow(row)

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
        
        self.csv_file.close()  # Cierra archivo CSV correctamente
        print("Archivo CSV cerrado.")
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
    def get_arm_positions_from_user_or_file(self):
        print("\n¿Deseas ingresar posiciones manualmente o cargar desde archivo?")
        print("1. Ingresar manualmente")
        print("2. Cargar desde archivo .txt")
        choice = input("Seleccione una opción (1/2): ").strip()

        if choice == "1":
            return self.get_user_joint_positions()
        elif choice == "2":
            file_path = input("Ingrese la ruta del archivo .txt: ").strip()
            try:
                with open(file_path, 'r') as file:
                    positions = {}
                    for line in file:
                        joint, value = line.strip().split()
                        positions[joint] = float(value)
                    return positions
            except Exception as e:
                print(f" Error al leer el archivo: {e}")
                return None
        else:
            print("Opción inválida. Cancelando entrada.")
            return None

    def run_sequence(self):
        """Ejecuta la secuencia de movimientos con opciones dinámicas del usuario."""
        input("\nMoviendo a posición cero... Presione Enter para continuar.")
        self.move_to({joint: 0.0 for joint in self.arm_joints})
        self.ask_walk_after_arm_motion()  #  Esta es la clave

        while True:
            print("\nOpciones:")
            print("1. Ingresar nueva posición objetivo")
            print("2. Volver a posición cero")
            
            option = input("Seleccione una opción (1/2): ").strip()
            
            if option == "1":
                target_positions = self.get_arm_positions_from_user_or_file()

                if target_positions is None:
                    print("\nCancelando la secuencia.")
                    continue

                self.move_to(target_positions)
                self.ask_walk_after_arm_motion()  # Vuelve a preguntar luego de cada movimiento

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


    def execute_trajectory_sequence(self, movimientos):
        """
        Ejecuta una secuencia de movimientos previamente definida.

        Parámetro:
            movimientos: lista de tuplas (x_vel, y_vel, yaw_vel, duración)
        """
        if not movimientos:
            print(" La secuencia está vacía. No se ejecutará ningún movimiento.")
            return

        # Mostrar resumen
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
            
    def get_walk_trajectory_from_user_or_file(self):
        direction_map = {
            "adelante":  (self.DEFAULT_X_VEL, 0.0, 0.0),
            "atras":     (-self.DEFAULT_X_VEL, 0.0, 0.0),
            "izquierda": (0.0, self.DEFAULT_Y_VEL , 0.0),
            "derecha":   (0.0, -self.DEFAULT_Y_VEL , 0.0),
            #"rotar":     (0.0, 0.0, 0.4),
            # Si quieres rotar izq o der separado:
            "rotar_izq":  (0.0, 0.0,  self.DEFAULT_YAW_VEL),
            "rotar_der":  (0.0, 0.0, -self.DEFAULT_YAW_VEL),
        }
        

        print("\n¿Deseas ingresar la trayectoria manualmente o cargar desde archivo?")
        print("1. Ingresar manualmente")
        print("2. Cargar desde archivo .txt")
        choice = input("Seleccione una opción (1/2): ").strip()

        if choice == "1":
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

        elif choice == "2":
            file_path = input("Ingrese la ruta del archivo .txt: ").strip()
            try:
                with open(file_path, 'r') as file:
                    trajectory = []
                    for line in file:
                        parts = line.strip().split()
                        if len(parts) == 2:
                            direction, duration = parts
                            direction = direction.lower()
                            if direction in direction_map:
                                x, y, yaw = direction_map[direction]
                                trajectory.append((x, y, yaw, float(duration)))
                            else:
                                print(f"Dirección inválida en archivo: '{direction}' — ignorando.")
                    return trajectory
            except Exception as e:
                print(f" Error al leer el archivo: {e}")
                return []
        else:
            print("Opción inválida. Cancelando entrada.")
            return []

            
    def move(self, client, x_vel=0.0, y_vel=0.0, yaw_vel=0.0, duration=2.0):
        """Envía un comando de movimiento al robot con velocidades específicas durante un tiempo determinado."""
        print(f"\n➡️ Iniciando movimiento: x_vel={x_vel}, y_vel={y_vel}, yaw_vel={yaw_vel}, duración={duration}s")
        self.client.Move(x_vel, y_vel, yaw_vel, True)
        time.sleep(duration)
        self.client.Move(0, 0, 0)  # Detener robot
        print("Movimiento finalizado.\n")
        time.sleep(0.8)

        
    def ask_walk_after_arm_motion(self):
        """Pregunta si se desea ejecutar una trayectoria después de un movimiento de brazos."""
        choice = input("\n¿Deseas que el robot ejecute una trayectoria ahora? (s/n): ").strip().lower()
        if choice == 's':
            trajectory = self.get_walk_trajectory_from_user_or_file()
            self.execute_trajectory_sequence(trajectory) # Llama a la función que define y ejecuta la secuencia
        else:
            print(" Caminata omitida.")


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

    #Inicialización cliente de alto nivel
    custom = Custom()
    custom.Init()
    custom.Start()

