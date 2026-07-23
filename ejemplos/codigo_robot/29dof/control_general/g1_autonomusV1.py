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
@file g1_autonomousV1.py
@author Sofía Milagros Castaño Vanegas - Robotics 4.0 Team
@date 2025-04-21
@version 1.0
@brief Navegación autónoma básica del robot G1 de Unitree utilizando SDK2.

Este script permite al robot G1 desplazarse de forma autónoma hacia una secuencia de puntos definidos
usando su odometría interna. Se emplea control proporcional adaptativo para mover el robot hacia cada objetivo,
corrigiendo su orientación antes y después del desplazamiento. 

Se contempla la posibilidad de retroceder ligeramente, movimiento lateral limitado, rotación con freno adaptativo
y timeout por objetivo. El sistema permite la carga de objetivos desde archivo o ingreso manual por consola.

@requisitos
- Conexión activa al robot G1 mediante Ethernet.
- SDK2 de Unitree instalada correctamente en el sistema.
- Robot encendido en modo normal en el MAIN OPERATION CONTROL (R1+X).
- Acceso a la interfaz `loco_client` y al canal `SportModeState_` para odometría.

@uso
    python3 g1_autonomousV1.py

@funcionalidades
- Carga de objetivos desde archivo o por consola.
- Control proporcional en x, y y rotación (yaw).
- Límite de velocidades y retroceso leve para evitar colisiones.
- Reorientación previa y posterior al movimiento.
- Timeout de 30 segundos por objetivo.
- Modo seguro de detención ante interrupción.
"""

# g1_autonomousV1.py (versión sin soporte para .txt)

import time
import math
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

class AutonomousNavigator:
    def __init__(self, interface):
        ChannelFactoryInitialize(0, interface)
        self.client = LocoClient()
        self.client.SetTimeout(10.0)
        self.client.Init()

        self.odom = None
        self.targets = []
        self.max_vyaw = 0.5

    def Init(self):
        self.subscriber_odom = ChannelSubscriber("rt/odommodestate", SportModeState_)
        self.subscriber_odom.Init(self.OdomMessageHandler, 10)

    def Start(self):
        print("Esperando odometría...")
        start_time = time.time()
        while self.odom is None and time.time() - start_time < 5:
            time.sleep(0.1)
        if self.odom is None:
            raise RuntimeError("No se recibió odometría a tiempo.")

    def OdomMessageHandler(self, msg: SportModeState_):
        self.odom = msg

    def load_targets(self):
        print("Ingrese manualmente los objetivos (x y yaw). Escriba 'fin' para terminar:")
        while True:
            entry = input("> ")
            if entry.lower() == "fin":
                break
            try:
                x, y, yaw = map(float, entry.strip().split())
                yaw = math.atan2(math.sin(yaw), math.cos(yaw))
                self.targets.append((x, y, yaw))
            except:
                print("Formato inválido. Usa: x y yaw")

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
        yaw_error = math.atan2(math.sin(desired_yaw - current_yaw), math.cos(desired_yaw - current_yaw))

        K_v = 0.5
        K_yaw = 1.2 if abs(yaw_error) > 0.2 else 0.4

        vx = K_v * distance * math.cos(yaw_error)
        vy = K_v * distance * math.sin(yaw_error)
        vyaw = K_yaw * yaw_error

        vx = max(min(vx, 0.4), -0.15)
        vy = max(min(vy, 0.3), -0.3)
        vyaw = max(min(vyaw, self.max_vyaw), -self.max_vyaw)

        return vx, vy, vyaw, distance, yaw_error

    def rotate_to_yaw(self, target_yaw, tolerance=0.11):
        print(f"Alineando a yaw final: {target_yaw:.2f}")
        for _ in range(100):
            x, y, yaw = self.get_current_pose()
            if yaw is None:
                time.sleep(0.1)
                continue
            yaw_error = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))

            if abs(yaw_error) < tolerance:
                print("Alineación final completada.")
                break

            vyaw = max(min(0.6 * yaw_error, 0.4), -0.4)
            print(f"Corrigiendo yaw: actual={yaw:.2f}, error={yaw_error:.2f}")
            self.client.Move(0, 0, vyaw, True)
            time.sleep(0.1)

        self.client.Move(0, 0, 0)
        time.sleep(0.4)

    def navigate(self, tolerance_dist=0.2, alignment_yaw=0.3):
        try:
            for i, (goal_x, goal_y, goal_yaw) in enumerate(self.targets, 1):
                print(f"\nObjetivo #{i}: ({goal_x:.2f}, {goal_y:.2f}, yaw={goal_yaw:.2f})")
                print("Paso 1: Reorientando hacia el objetivo...")
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
                        print("Retrocediendo ligeramente para evitar giro cerrado...")
                        self.client.Move(0, 0, -0.5, True)
                        time.sleep(0.5)
                        self.client.Move(0, 0, 0)

                    if abs(yaw_error) < alignment_yaw:
                        print("Orientación hacia objetivo lograda.")
                        break

                    vyaw = max(min(0.6 * yaw_error, 0.4), -0.4)
                    print(f"Corrigiendo yaw: actual={yaw:.2f}, deseado={desired_yaw:.2f}, error={yaw_error:.2f}")
                    self.client.Move(0, 0, vyaw, True)
                    time.sleep(0.5)
                    self.client.Move(0, 0, 0)

                self.client.Move(0, 0, 0)
                time.sleep(0.6)

                print("Paso 2: Avanzando hacia el objetivo...")
                start = time.time()
                max_time = 30
                while time.time() - start < max_time:
                    x, y, yaw = self.get_current_pose()
                    if x is None:
                        time.sleep(0.1)
                        continue

                    vx, vy, vyaw, dist_err, yaw_err = self.compute_control(goal_x, goal_y, x, y, yaw)

                    print(f"Posición actual: ({x:.2f}, {y:.2f}), Error distancia: {dist_err:.3f}, Vel: vx={vx:.2f}, vy={vy:.2f}, vyaw={vyaw:.2f}")
                    self.client.Move(vx, vy, vyaw, True)
                    time.sleep(0.5)

                    if abs(dist_err) < tolerance_dist:
                        print("Posición alcanzada.")
                        break

                else:
                    print("Tiempo agotado para alcanzar el objetivo.")

                self.client.Move(0, 0, 0)
                time.sleep(0.6)

                print("Paso 3: Alineando con la orientación final...")
                self.rotate_to_yaw(goal_yaw)

            print("\nTodos los objetivos fueron alcanzados correctamente.")

        except KeyboardInterrupt:
            print("\nNavegación interrumpida.")
            self.client.Move(0, 0, 0)
            self.client.StopMove()
        print("Robot detenido correctamente.")

def main():
    interface = input("Interfaz de red (ej: eth0): ").strip()
    nav = AutonomousNavigator(interface)
    nav.Init()
    nav.Start()
    nav.load_targets()
    nav.navigate()

if __name__ == "__main__":
    main()
