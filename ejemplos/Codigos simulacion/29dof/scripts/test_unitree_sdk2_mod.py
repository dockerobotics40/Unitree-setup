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
# Este script:
#   - Se suscribe al canal `rt/lowstate` para recibir estados (IMU, articulaciones).
#   - Publica comandos `LowCmd` en el canal `rt/lowcmd` para controlar motores.
#   - Envía un torque constante (τ = 1.0) a cada articulación como demostración.
#
# Nota:
#   - Este ejemplo es puramente ilustrativo, no representa un controlador real.
#   - El robot G1 posee 29 motores, por lo cual el bucle recorre todos.
#   - En simulación, el "elastic band" debe estar activado (config.yaml).
# -----------------------------------------------------------------------------
import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC


def LowStateHandler(msg: LowState_):
    print("IMU state: ", msg.imu_state)

if __name__ == "__main__":
    ChannelFactoryInitialize(1, "lo")
    low_state_suber = ChannelSubscriber("rt/lowstate", LowState_)
    low_state_suber.Init(LowStateHandler, 10)

    low_cmd_puber = ChannelPublisher("rt/lowcmd", LowCmd_)
    low_cmd_puber.Init()
    crc = CRC()

    cmd = unitree_hg_msg_dds__LowCmd_()
    cmd.mode_pr = 0  # PR mode (Pitch/Roll)
    cmd.mode_machine = 0
    for i in range(29):  # HG robot has 29 motors instead of 20
        cmd.motor_cmd[i].mode = 1  # 1:Enable, 0:Disable
        cmd.motor_cmd[i].q = 0.0
        cmd.motor_cmd[i].kp = 0.0
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].kd = 0.0
        cmd.motor_cmd[i].tau = 0.0
        
    while True:
        for i in range(29):  # Update all motors instead of just 12
            cmd.motor_cmd[i].q = 0.0 
            cmd.motor_cmd[i].kp = 0.0
            cmd.motor_cmd[i].dq = 0.0 
            cmd.motor_cmd[i].kd = 0.0
            cmd.motor_cmd[i].tau = 1.0 
        
        cmd.crc = crc.Crc(cmd)

        #Publish message
        low_cmd_puber.Write(cmd)
        time.sleep(0.002)