"""
@file g1_autonomousV1.py
@author Sofía Milagros Castaño Vanegas - Robotics 4.0 Team
@date 2025-04-21
@version 1.0
@brief Navegación autónoma básica del robot G1 de Unitree utilizando SDK2.

Este script permite al robot G1 desplazarse de forma autónoma hacia una secuencia de puntos definidos
usando su odometría interna. Se emplea control proporcional adaptativo para mover el robot hacia cada objetivo,
corrigiendo su orientación antes y después del desplazamiento, en cada posición esta la posibilidad de
ejecutar una posición de las articulaciones superiores. 

Se contempla la posibilidad de retroceder ligeramente, movimiento lateral limitado, rotación con freno adaptativo
y timeout por objetivo. El sistema permite la carga de objetivos desde archivo o ingreso manual por consola.

@requisitos
- Conexión activa al robot G1 mediante Ethernet.
- SDK2 de Unitree instalada correctamente en el sistema.
- Robot encendido en modo normal en el MAIN OPERATION CONTROL (R1+X).
- Acceso a la interfaz `loco_client` y al canal `SportModeState_` para odometría.

@uso
    python3 g1_autonomousWithArmV1.py

@funcionalidades
- Carga de objetivos desde archivo o por consola.
- Control proporcional en x, y y rotación (yaw).
- Límite de velocidades y retroceso leve para evitar colisiones.
- Reorientación previa y posterior al movimiento.
- Timeout de 30 segundos por objetivo.
- Modo seguro de detención ante interrupción.
"""

import time
import math
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

class G1JointIndex:
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

class AutonomousNavigatorWithArm:
    def __init__(self, interface):
        ChannelFactoryInitialize(0, interface)
        #Inicializaciones del brazo
        self.control_dt = 0.02
        self.kp = 60.0
        self.kd = 1.5
        self.crc = CRC()
        self.t = 0.0
        self.T = 4.0
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.first_update = False
        self.target_pos = {}
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

        self.client = LocoClient()
        self.client.SetTimeout(10.0)
        self.client.Init()

        self.odom = None
        self.targets = []
        self.max_vyaw = 0.5

        
    def Init(self):
        self.subscriber_odom = ChannelSubscriber("rt/odommodestate",SportModeState_)
        self.subscriber_odom.Init(self.OdomMessageHandler, 10)
        self.publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.publisher.Init()
        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self.LowStateHandler, 10)
        
    def Start(self):
        # Espera inicial de odometría
        print(" Esperando odometría...")
        start_time = time.time()
        while self.odom is None and time.time() - start_time < 5:
            time.sleep(0.1)
        if self.odom is None:
            raise RuntimeError("No se recibió odometría a tiempo.")
        self.thread = RecurrentThread(interval=self.control_dt, target=self.LowCmdWrite, name="arm_control")
        while not self.first_update:
            time.sleep(0.1)
        self.thread.Start()
        
    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg
        if not self.first_update:
            self.first_update = True
            
    def interpolate_position(self, q_init, q_target):
        """Interpolación de posición para un movimiento suave."""
        ratio = (1 - math.cos(math.pi * (self.t / self.T))) / 2 if self.t < self.T else 1.0
        return q_init + (q_target - q_init) * ratio
    
    def LowCmdWrite(self):
        if self.low_state is None:
            return

        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1  # Enable arm_sdk

        for joint in self.arm_joints:
            q_init = self.low_state.motor_state[joint].q
            q_target = self.target_pos.get(joint, q_init)
            pos = self.interpolate_position(q_init, q_target)
            self.low_cmd.motor_cmd[joint].q = pos
            self.low_cmd.motor_cmd[joint].dq = 0.0
            self.low_cmd.motor_cmd[joint].tau = 0.0
            self.low_cmd.motor_cmd[joint].kp = self.kp
            self.low_cmd.motor_cmd[joint].kd = self.kd

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.publisher.Write(self.low_cmd)
        self.t += self.control_dt

    def move_to(self, updates: dict, duration=1.0):
            self.target_pos.update(updates)
            self.T = duration
            self.t = 0.0

            # Esperar hasta completar interpolación
            while self.t < self.T:
                time.sleep(self.control_dt)
                
    def get_user_joint_positions(self, paso_num):
        """Retorna la configuración de articulaciones correspondiente al paso solicitado."""
        pasos = [
            # Paso 1
            {
                G1JointIndex.LeftShoulderPitch: 0.0,
                G1JointIndex.LeftShoulderRoll: 0.35,
                G1JointIndex.LeftShoulderYaw: 0.0,
                G1JointIndex.LeftElbow: 1.5,
                G1JointIndex.LeftWristRoll: 0.0,
                G1JointIndex.LeftWristPitch: 0.0,
                G1JointIndex.LeftWristYaw: 0.0,
                G1JointIndex.RightShoulderPitch: 0.0,
                G1JointIndex.RightShoulderRoll: -0.35,
                G1JointIndex.RightShoulderYaw: 0.0,
                G1JointIndex.RightElbow: 1.5,
                G1JointIndex.RightWristRoll: 0.0,
                G1JointIndex.RightWristPitch: 0.0,
                G1JointIndex.RightWristYaw: 0.0,
                G1JointIndex.WaistYaw: 0.0,
                G1JointIndex.WaistRoll: 0.0,
                G1JointIndex.WaistPitch: 0.0
            },
            # Paso 2
            {
                G1JointIndex.LeftShoulderPitch: 0.0,
                G1JointIndex.LeftShoulderRoll: 0.35,
                G1JointIndex.LeftShoulderYaw: 0.0,
                G1JointIndex.LeftElbow: 0.0,
                G1JointIndex.LeftWristRoll: 0.0,
                G1JointIndex.LeftWristPitch: 0.0,
                G1JointIndex.LeftWristYaw: 0.0,
                G1JointIndex.RightShoulderPitch: 0.0,
                G1JointIndex.RightShoulderRoll: -0.35,
                G1JointIndex.RightShoulderYaw: 0.0,
                G1JointIndex.RightElbow: 0.0,
                G1JointIndex.RightWristRoll: 0.0,
                G1JointIndex.RightWristPitch: 0.0,
                G1JointIndex.RightWristYaw: 0.0,
                G1JointIndex.WaistYaw: 0.0,
                G1JointIndex.WaistRoll: 0.0,
                G1JointIndex.WaistPitch: 0.0
            },
            # Paso 3
            {
                G1JointIndex.LeftShoulderPitch: 0.0,
                G1JointIndex.LeftShoulderRoll: 0.35,
                G1JointIndex.LeftShoulderYaw: 0.4,
                G1JointIndex.LeftElbow: 0.0,
                G1JointIndex.LeftWristRoll: 0.0,
                G1JointIndex.LeftWristPitch: 0.0,
                G1JointIndex.LeftWristYaw: 0.0,
                G1JointIndex.RightShoulderPitch: 0.0,
                G1JointIndex.RightShoulderRoll: -0.35,
                G1JointIndex.RightShoulderYaw: -0.4,
                G1JointIndex.RightElbow: 0.0,
                G1JointIndex.RightWristRoll: -1.0,
                G1JointIndex.RightWristPitch: 0.0,
                G1JointIndex.RightWristYaw: 0.0,
                G1JointIndex.WaistYaw: 0.0,
                G1JointIndex.WaistRoll: 0.0,
                G1JointIndex.WaistPitch: 0.0
            },
            # Paso 4
            {
                G1JointIndex.LeftShoulderPitch: 0.0,
                G1JointIndex.LeftShoulderRoll: 0.3,
                G1JointIndex.LeftShoulderYaw: -0.1,
                G1JointIndex.LeftElbow: 0.0,
                G1JointIndex.LeftWristRoll: 0.0,
                G1JointIndex.LeftWristPitch: 0.0,
                G1JointIndex.LeftWristYaw: 0.0,
                G1JointIndex.RightShoulderPitch: 0.0,
                G1JointIndex.RightShoulderRoll: -0.3,
                G1JointIndex.RightShoulderYaw: 0.1,
                G1JointIndex.RightElbow: 0.0,
                G1JointIndex.RightWristRoll: -1.0,
                G1JointIndex.RightWristPitch: 0.0,
                G1JointIndex.RightWristYaw: 0.0,
                G1JointIndex.WaistYaw: 0.0,
                G1JointIndex.WaistRoll: 0.0,
                G1JointIndex.WaistPitch: 0.0
            },
            # Paso 5
            {
                G1JointIndex.LeftShoulderPitch: 0.0,
                G1JointIndex.LeftShoulderRoll: 0.35,
                G1JointIndex.LeftShoulderYaw: 0.4,
                G1JointIndex.LeftElbow: 0.0,
                G1JointIndex.LeftWristRoll: 0.0,
                G1JointIndex.LeftWristPitch: 0.0,
                G1JointIndex.LeftWristYaw: 0.0,
                G1JointIndex.RightShoulderPitch: 0.0,
                G1JointIndex.RightShoulderRoll: -0.35,
                G1JointIndex.RightShoulderYaw: -0.4,
                G1JointIndex.RightElbow: 0.0,
                G1JointIndex.RightWristRoll: -1.0,
                G1JointIndex.RightWristPitch: 0.0,
                G1JointIndex.RightWristYaw: 0.0,
                G1JointIndex.WaistYaw: 0.0,
                G1JointIndex.WaistRoll: 0.0,
                G1JointIndex.WaistPitch: 0.0
            },
            # Paso 6
            {
                G1JointIndex.LeftShoulderPitch: 0.0,
                G1JointIndex.LeftShoulderRoll: 0.35,
                G1JointIndex.LeftShoulderYaw: 0.0,
                G1JointIndex.LeftElbow: 1.5,
                G1JointIndex.LeftWristRoll: 0.0,
                G1JointIndex.LeftWristPitch: 0.0,
                G1JointIndex.LeftWristYaw: 0.0,
                G1JointIndex.RightShoulderPitch: 0.0,
                G1JointIndex.RightShoulderRoll: -0.35,
                G1JointIndex.RightShoulderYaw: 0.0,
                G1JointIndex.RightElbow: 1.5,
                G1JointIndex.RightWristRoll: 0.0,
                G1JointIndex.RightWristPitch: 0.0,
                G1JointIndex.RightWristYaw: 0.0,
                G1JointIndex.WaistYaw: 0.0,
                G1JointIndex.WaistRoll: 0.0,
                G1JointIndex.WaistPitch: 0.0
            },
        ]

        if 1 <= paso_num <= len(pasos):
            return pasos[paso_num - 1]
        else:
            raise ValueError(f"Número de paso inválido. Debe estar entre 1 y {len(pasos)}.")


    def OdomMessageHandler(self, msg: SportModeState_):
        self.odom = msg

    def load_targets(self, file_path=None):
        if file_path:
            with open(file_path, "r") as f:
                for line in f:
                    x, y, yaw = map(float, line.strip().split())
                    yaw = math.atan2(math.sin(yaw), math.cos(yaw))  # Normaliza yaw
                    self.targets.append((x, y, yaw))
        else:
            print("¿Deseas ingresar objetivos manualmente? (s/n):")
            if input("> ").lower() == 's':
                print("Ingresa los puntos objetivo (x y yaw). Escribe 'fin' para terminar.")
                while True:
                    entry = input("> ")
                    if entry.lower() == "fin":
                        break
                    try:
                        x, y, yaw = map(float, entry.strip().split())
                        yaw = math.atan2(math.sin(yaw), math.cos(yaw))  # Normaliza
                        self.targets.append((x, y, yaw))
                    except:
                        print("Formato inválido. Usa: x y yaw")
            else:
                print("Usando lista por defecto")
                #Esta lista puede ser cambiada
                self.targets = [
                    (0.5, 0.0, 0.0),
                    (1.0, 0.5, math.pi/2),
                    (1.0, 1.0, math.pi),
                ]

    def get_current_pose(self):
        if self.odom is None:
            return None, None, None
        x, y, _ = self.odom.position
        yaw = self.odom.imu_state.rpy[2]
        return x, y, yaw

    def compute_control(self, goal_x, goal_y, current_x, current_y, current_yaw):
        dx = goal_x - current_x
        dy = goal_y - current_y
        distance = math.hypot(dx, dy)

        desired_yaw = math.atan2(dy, dx)
        yaw_error = desired_yaw - current_yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        K_v = 0.5
        K_yaw = 1.2 if abs(yaw_error) > 0.2 else 0.4  # "P con freno"

        vx = K_v * distance * math.cos(yaw_error)
        vy = K_v * distance * math.sin(yaw_error)
        vyaw = K_yaw * yaw_error

        vx = max(min(vx, 0.4), -0.15)               # Permite retroceder un poco
        vy = max(min(vy, 0.3), -0.3)                # Movimiento lateral limitado
        vyaw = max(min(vyaw, self.max_vyaw), -self.max_vyaw)

        return vx, vy, vyaw, distance, yaw_error

    def rotate_to_yaw(self, target_yaw, tolerance=0.11):
        print(f" Alineando a yaw final: {target_yaw:.2f}")
        for _ in range(100):  # máximo 10 segundos
            x, y, yaw = self.get_current_pose()
            if yaw is None:
                time.sleep(0.1)
                continue
            yaw_error = target_yaw - yaw
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

            if abs(yaw_error) < tolerance:
                print(" Alineación final completada.")
                break

            vyaw = 0.6 * yaw_error
            vyaw = max(min(vyaw, 0.4), -0.4)

            print(f"Corrigiendo yaw: actual={yaw:.2f}, error={yaw_error:.2f}")
            self.client.Move(0, 0, vyaw, True)
            time.sleep(0.1)

        self.client.Move(0, 0, 0)
        time.sleep(0.4)


    def navigate(self, tolerance_dist=0.2, alignment_yaw=0.3):
        try:
            for i, (goal_x, goal_y, goal_yaw) in enumerate(self.targets, 1):
                print(f"\n Objetivo #{i}: ({goal_x:.2f}, {goal_y:.2f}, yaw={goal_yaw:.2f})")

                # Paso 1: Orientar hacia objetivo
                print(" Paso 1: Reorientando hacia el objetivo...")
                while True:
                    x, y, yaw = self.get_current_pose()
                    if x is None:
                        time.sleep(0.1)
                        continue

                    dx = goal_x - x
                    dy = goal_y - y
                    desired_yaw = math.atan2(dy, dx)
                    yaw_error = math.atan2(math.sin(desired_yaw - yaw), math.cos(desired_yaw - yaw))

                    if abs(yaw_error) > 0.78:
                        print(" Retrocediendo ligeramente para evitar giro cerrado...")
                        self.client.Move(0.0, 0, -0.5, True)
                        time.sleep(0.5)
                        self.client.Move(0, 0, 0)

                    if abs(yaw_error) < alignment_yaw:
                        print(" Orientación hacia objetivo lograda.")
                        break

                    vyaw = 0.6 * yaw_error
                    vyaw = max(min(vyaw, 0.4), -0.4)

                    print(f" Corrigiendo yaw: actual={yaw:.2f}, deseado={desired_yaw:.2f}, error={yaw_error:.2f}")
                    self.client.Move(0, 0, vyaw, True)
                    time.sleep(0.5)
                    self.client.Move(0, 0, 0)

                self.client.Move(0, 0, 0)
                time.sleep(0.6)

                # Paso 2: Avanzar hacia el punto
                print(" Paso 2: Avanzando hacia el objetivo...")
                start = time.time()
                max_time = 30
                while time.time() - start < max_time:
                    x, y, yaw = self.get_current_pose()
                    if x is None:
                        time.sleep(0.5)
                        continue

                    vx, vy, vyaw, dist_err, yaw_err = self.compute_control(goal_x, goal_y, x, y, yaw)

                    print(f" Posición actual: ({x:.2f}, {y:.2f}), Error distancia: {dist_err:.3f}, Vel: vx={vx:.2f}, vy={vy:.2f}, vyaw={vyaw:.2f}")
                    self.client.Move(vx, vy, vyaw, True)
                    time.sleep(0.1)

                    if abs(dist_err) < tolerance_dist:
                        if abs(dist_err) < 0.10:
                            print("Posición alcanzada con alta precisión.")
                        else:
                            print("Posición suficientemente cercana: 20 cm de vector normal de diferencia")
                        break
                else:
                    print(" Tiempo agotado para alcanzar el objetivo.")

                self.client.Move(0, 0, 0)
                time.sleep(0.6)

                # Paso 3: Alinear a yaw objetivo
                print(" Paso 3: Alineando con la orientación final...")
                self.rotate_to_yaw(goal_yaw)
                
                respuesta = input("¿Deseas ejecutar un movimiento de brazos en este objetivo? (s/n): ").strip().lower()
    
                if respuesta == 's':
                    try:
                        paso = int(input("¿Qué paso deseas ejecutar? (1-6): ").strip())
                        config = self.get_user_joint_positions(paso)
                        print(f"Ejecutando movimiento de brazos del paso {paso}...")
                        self.move_to(config, duration=2)
                        print("Se alcanzó la posición objetivo de los brazos")
                        otrarespuesta = input("¿Deseas ejecutar otro movimiento de brazos en este objetivo? (s/n): ").strip().lower()
                        if otrarespuesta == 's':
                            paso2 = int(input("¿Qué paso deseas ejecutar? (1-6): ").strip())
                            config2 = self.get_user_joint_positions(paso2)
                            print(f"Ejecutando movimiento de brazos del paso {paso2}...")
                            self.move_to(config2, duration=2)
                            print("Se alcanzó la posición objetivo de los brazos")
                            
                    except ValueError:
                        print("Entrada inválida. Se esperaba un número entero.")
                    except Exception as e:
                        print(f"Error al ejecutar movimiento: {e}")
                else: 
                    print("\n Caminando hacia siguiente punto")

            print("\n Todos los objetivos alcanzados y orientados correctamente.")

        except KeyboardInterrupt:
            print("\n Navegación interrumpida por el usuario.")
            self.client.Move(0, 0, 0)
            self.client.StopMove()
        print(" Robot detenido correctamente.")



def main():
    interface = input("Interfaz de red (ej: eth0): ").strip()
    nav = AutonomousNavigatorWithArm(interface)
    nav.Init()
    nav.Start()

    opc = input("¿Cargar puntos desde archivo? (s/n): ").strip().lower()
    if opc == 's':
        path = input("Ruta del archivo: ").strip()
        nav.load_targets(path)
    else:
        nav.load_targets()

    nav.navigate()

if __name__ == "__main__":
    main()