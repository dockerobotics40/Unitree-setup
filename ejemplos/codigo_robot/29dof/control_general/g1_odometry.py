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
@file g1_odometry.py
@author Sofía Milagros Castaño Vanegas - Robotics 4.0 Team
@date 2025-04-21
@version 1.0
@brief NRegistro de odometría

Este script permite captar los datos de odometría en posición y orientación del robot G1. 

Se contempla la posibilidad de retroceder ligeramente, movimiento lateral limitado, rotación con freno adaptativo
y timeout por objetivo. El sistema permite la carga de objetivos desde archivo o ingreso manual por consola.

@requisitos
- Conexión activa al robot G1 mediante Ethernet.
- SDK2 de Unitree instalada correctamente en el sistema.
- Robot encendido en modo normal.
- Acceso a la interfaz `loco_client` y al canal `SportModeState_` para odometría.

@uso
    python3 g1_odometry.py <nombreInterfaz> 

    - <nombreInterfaz>: nombre de la interfaz de red conectada al robot (ej. 'eth0', 'enp0s31f6').

"""

import sys
import time
import math

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_, IMUState_



class OdomRegister:
    def __init__(self):
        self.low_state = None
        self.first_update = False
        self.odom_state = None
        self.counter_ = 0

    def Init(self):
        self.subscriber_low = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber_low.Init(self.LowStateHandler, 10)
        self.subscriber_odom = ChannelSubscriber("rt/odommodestate",SportModeState_)
        self.subscriber_odom.Init(self.OdomMessageHandler, 10)
        
    def Start(self):
        while not self.first_update:
            time.sleep(0.1)
        print("Iniciando registro de odometría")
        

    def LowStateHandler(self, msg: LowState_):
        
        self.low_state = msg
        if not self.first_update:
            self.first_update = True      
            
    def OdomMessageHandler(self, msg: SportModeState_):
        self.odom_state = msg
        self.counter_ += 1
        if (self.counter_ % 500 == 0) :
            self.counter_ = 0
            pos = self.odom_state.position     # [x, y, z]
            vel = self.odom_state.velocity     # [vx, vy, vz]
            yaw_rate = self.odom_state.yaw_speed
            rpy = self.odom_state.imu_state.rpy  # [roll, pitch, yaw]
            roll, pitch, yaw = rpy

            print("\n ODOMETRÍA ACTUAL")
            print(f" Posición     -> x: {pos[0]:.3f}, y: {pos[1]:.3f}, z: {pos[2]:.3f}")
            print(f" Orientación  -> roll: {roll:.3f}, pitch: {pitch:.3f}, yaw: {yaw:.3f}")
            print(f" Yaw Vel      -> {yaw_rate:.3f} rad/s")
            print(f" Velocidad    -> vx: {vel[0]:.3f}, vy: {vel[1]:.3f}, vz: {vel[2]:.3f}")




def main():
    if len(sys.argv) < 2:
        sys.exit("Uso: python3 g1_odometry.py <interfaz_red>")

    ChannelFactoryInitialize(0, sys.argv[1])
    odom = OdomRegister()
    odom.Init()
    odom.Start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nFinalizando registro de odometría.")

if __name__ == "__main__":
    main()
