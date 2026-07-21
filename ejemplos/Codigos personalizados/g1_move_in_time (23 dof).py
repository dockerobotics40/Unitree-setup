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
@file g1_move_in_time (23 dof).py
@autor Sofía Milagros Castaño Vanegas - Robotics 4.0 Team
@date 2025-03-05
@version 1.1
@brief Script de control programado para un robot Unitree G1 EDU de 23 DoF sin manos.

Este script ejecuta una secuencia de locomoción preestablecida para el G1:
un desplazamiento en forma de cuadrado y un saludo corporal final. Utiliza
`LocoClient` de Unitree SDK2 para enviar comandos de velocidad.

La llamada `WaveHand()` fue retirada porque corresponde a una tarea integrada
de brazos y su disponibilidad depende de la configuración y del firmware del
robot. El saludo final se realiza únicamente mediante locomoción, por lo que
no requiere manos ni control articular de los brazos.

@requisitos
- Conexión Ethernet activa entre el PC y el G1.
- Unitree SDK2 Python instalada y configurada.
- Robot encendido, de pie y en modo normal de locomoción.
- Servicio de locomoción del robot activo.
- Área de trabajo amplia, plana y libre de obstáculos.

@uso
    python3 "g1_move_in_time (23 dof).py" <nombreInterfaz>

@funcionalidades
- Movimiento en cuadrado mediante avance y giro.
- Saludo corporal final compatible con el G1 de 23 DoF sin manos.
- Detención segura al terminar, ante una interrupción o ante un error.
"""

import sys
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient


# Configuración de velocidades y tiempos.
FORWARD_SPEED = 0.4          # Velocidad de avance en m/s.
ROTATION_SPEED = 0.5         # Velocidad de giro en rad/s.
FORWARD_DURATION = 3.0       # Duración de cada avance en segundos.
TURN_DURATION = 3.14         # 0.5 rad/s × 3.14 s ≈ 1.57 rad ≈ 90°.
PAUSE_BETWEEN_MOVES = 1.0    # Pausa después de cada movimiento.

# Parámetros del saludo corporal final.
GREETING_ROTATION_SPEED = 0.35
GREETING_SHORT_DURATION = 0.6
GREETING_LONG_DURATION = 1.2
GREETING_PAUSE = 0.3


def initialize_robot(network_interface):
    """
    Inicializa la comunicación con el robot y crea el cliente de locomoción.

    El robot debe encontrarse previamente encendido, de pie y en modo normal
    de locomoción. Esta función no cambia automáticamente su postura ni activa
    modos mediante el control remoto.

    Args:
        network_interface (str): Interfaz de red usada para comunicarse
            con el robot, por ejemplo, "enp2s0".

    Returns:
        LocoClient: Cliente de locomoción inicializado.
    """
    ChannelFactoryInitialize(0, network_interface)

    client = LocoClient()
    client.SetTimeout(10.0)
    client.Init()

    print("Conexión con el robot inicializada.")
    print("Robot listo para moverse cuando el área sea segura.")
    return client


def stop_robot(client):
    """
    Envía comandos redundantes de detención al robot.

    Args:
        client (LocoClient): Cliente de locomoción inicializado.
    """
    client.Move(0.0, 0.0, 0.0)
    client.StopMove()


def move(
    client,
    x_vel=0.0,
    y_vel=0.0,
    yaw_vel=0.0,
    duration=1.0,
    pause=PAUSE_BETWEEN_MOVES,
):
    """
    Mantiene una velocidad durante un intervalo y luego detiene el robot.

    Args:
        client (LocoClient): Cliente que ejecuta el movimiento.
        x_vel (float): Velocidad longitudinal en m/s.
        y_vel (float): Velocidad lateral en m/s.
        yaw_vel (float): Velocidad angular en rad/s.
        duration (float): Duración del movimiento en segundos.
        pause (float): Pausa posterior a la detención en segundos.

    Raises:
        ValueError: Si la duración o la pausa son negativas.
    """
    if duration < 0.0:
        raise ValueError("La duración del movimiento no puede ser negativa.")

    if pause < 0.0:
        raise ValueError("La pausa posterior no puede ser negativa.")

    client.Move(x_vel, y_vel, yaw_vel, True)
    time.sleep(duration)

    stop_robot(client)

    if pause > 0.0:
        time.sleep(pause)


def execute_square(client):
    """
    Ejecuta cuatro segmentos de avance y cuatro giros aproximados de 90°.

    El recorrido real puede diferir de un cuadrado perfecto por deslizamiento,
    irregularidades del piso, estado de calibración y respuesta dinámica.
    """
    print("Iniciando reto: movimiento en cuadrado...")

    for side in range(1, 5):
        print(f"Lado {side}/4: avanzando...")
        move(
            client,
            x_vel=FORWARD_SPEED,
            duration=FORWARD_DURATION,
        )

        print(f"Giro {side}/4: rotando aproximadamente 90 grados...")
        move(
            client,
            yaw_vel=-ROTATION_SPEED,
            duration=TURN_DURATION,
        )


def execute_body_greeting(client):
    """
    Ejecuta un saludo corporal sin usar manos ni tareas integradas de brazos.

    La secuencia realiza tres giros breves cuyas rotaciones se compensan
    aproximadamente, de modo que el robot finalice con una orientación cercana
    a la que tenía antes del saludo.
    """
    print("Ejecutando saludo corporal compatible con 23 DoF...")

    move(
        client,
        yaw_vel=GREETING_ROTATION_SPEED,
        duration=GREETING_SHORT_DURATION,
        pause=GREETING_PAUSE,
    )
    move(
        client,
        yaw_vel=-GREETING_ROTATION_SPEED,
        duration=GREETING_LONG_DURATION,
        pause=GREETING_PAUSE,
    )
    move(
        client,
        yaw_vel=GREETING_ROTATION_SPEED,
        duration=GREETING_SHORT_DURATION,
        pause=GREETING_PAUSE,
    )


def main():
    """
    Verifica la interfaz de red, solicita confirmación de seguridad, inicializa
    el cliente y ejecuta el movimiento en cuadrado y el saludo corporal.
    """
    if len(sys.argv) != 2:
        print(f'Uso: python3 "{sys.argv[0]}" <nombreInterfaz>')
        sys.exit(1)

    network_interface = sys.argv[1]
    client = None

    print("\nADVERTENCIA DE SEGURIDAD")
    print("- Usa una superficie plana y no resbaladiza.")
    print("- Mantén personas, cables y obstáculos fuera del recorrido.")
    print("- Conserva acceso inmediato al control remoto y al paro de emergencia.")
    input("Presiona Enter únicamente cuando el área esté despejada...")

    try:
        client = initialize_robot(network_interface)

        input(
            "Presiona Enter cuando el G1 esté de pie, estable "
            "y listo para iniciar..."
        )

        execute_square(client)
        execute_body_greeting(client)

        stop_robot(client)
        print("\nRutina terminada correctamente.")

    except KeyboardInterrupt:
        print("\nPrograma interrumpido por el usuario.")

    except Exception as error:
        print(f"\nError durante la ejecución: {error}")

    finally:
        if client is not None:
            try:
                stop_robot(client)
                print("Robot detenido correctamente.")
            except Exception as stop_error:
                print(f"Error al enviar la detención final: {stop_error}")


if __name__ == "__main__":
    main()
