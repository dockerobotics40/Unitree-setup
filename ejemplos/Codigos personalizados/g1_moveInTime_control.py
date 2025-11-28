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
@file g1_move_in_time.py
@autor Sofía Milagros Castaño Vanegas - Robotics 4.0 Team
@date 2025-03-05
@version 1.0
@brief Script de control programado para el robot G1 de Unitree.

Este script define una secuencia de movimientos preestablecidos para el robot G1, incluyendo un reto
de desplazamiento en forma de cuadrado y un gesto de saludo final. Utiliza la API `loco_client` 
de la SDK2 de Unitree para enviar comandos de locomoción y gestos.

@requisitos
- Conexión Ethernet activa entre el PC y el G1.
- Robot en modo normal (cero torque) al inicio.
- SDK2 configurada correctamente.

@uso
    python3 g1_move_in_time.py <nombreInterfaz>

@funcionalidades
- Movimiento en cuadrado (avance + giro).
- Saludo con la "mano" del robot.
"""


import sys
import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

# Configuración de velocidades y tiempos (ajustables según el escenario)
FORWARD_SPEED = 0.4    # Velocidad de avance en metros por segundo (m/s)
LATERAL_SPEED = 0.3    # Velocidad de desplazamiento lateral en m/s (no usado en este script)
ROTATION_SPEED = 0.5   # Velocidad de giro en radianes por segundo (rad/s)
DURATION = 2.0         # Duración predeterminada de cada movimiento en segundos

def initialize_robot(network_interface):
    """
    Inicializa la conexión con el robot, lo coloca en posición de pie
    y lo deja listo para ejecutar movimientos.

    Args:
        network_interface (str): Nombre de la interfaz de red utilizada para la comunicación.

    Returns:
        LocoClient: Instancia del cliente de locomoción del robot.
    """
    # Configuración del canal de comunicación con el robot
    ChannelFactoryInitialize(0, network_interface)
    client = LocoClient()
    client.SetTimeout(10.0)  # Establece el tiempo máximo de espera de respuesta
    client.Init()  # Inicializa el cliente de locomoción

    # Secuencia de activación del robot
    print("Iniciando robot...")      

    print("Robot listo para moverse una vez este seguro y con espacio.")
    return client

def move(client, x_vel=0.0, y_vel=0.0, yaw_vel=0.0, duration=DURATION):
    """
    Envía un comando de movimiento al robot con velocidades específicas y
    mantiene el movimiento durante un tiempo determinado.

    Args:
        client (LocoClient): Cliente del robot que ejecutará el movimiento.
        x_vel (float): Velocidad en el eje X (adelante/atrás).
        y_vel (float): Velocidad en el eje Y (izquierda/derecha).
        yaw_vel (float): Velocidad de rotación sobre el eje Z.
        duration (float): Tiempo en segundos que el movimiento será mantenido.
    """
    client.Move(x_vel, y_vel, yaw_vel, True)  # Enviar comando de movimiento
    time.sleep(duration)  # Mantener el movimiento por el tiempo definido
    client.Move(0, 0, 0)  # Detener el robot tras la duración establecida
    time.sleep(1.0)  # Pequeña pausa antes de cualquier otro comando

def main():
    """
    Función principal del programa. Se encarga de:
    1. Verificar los argumentos de entrada.
    2. Pedir confirmación al usuario antes de iniciar.
    3. Inicializar el robot y ejecutar un reto de movimiento en cuadrado.
    4. Finalizar con un saludo del robot.
    """
    # Verifica que se haya pasado la interfaz de red como argumento
    if len(sys.argv) < 2:
        print(f"Uso: python3 {sys.argv[0]} networkInterface")
        sys.exit(1)

    print("\n Asegúrate de que no haya obstáculos alrededor del robot.")
    input("Presiona Enter cuando el área esté despejada...")

    try:
        client = initialize_robot(sys.argv[1])  # Inicializar el robot

        # RETO: Movimiento en cuadrado
        input("Presiona Enter cuando el robot este listo para iniciar")
        print("Iniciando reto: Movimiento en cuadrado...")
        for _ in range(4):  # Repetir 4 veces para completar el cuadrado
            move(client, x_vel=FORWARD_SPEED, duration=3.0)  # Avanza
            move(client, yaw_vel=-ROTATION_SPEED, duration=3.14)  # Gira 90° a la izquierda

        # RETO: Saludo final
        print("Finalizando con un saludo...")
        client.WaveHand()  # Ordena al robot realizar un gesto de saludo
        client.StopMove()  # Asegura que el robot se detenga completamente

    except KeyboardInterrupt:
        print("\nPrograma interrumpido por el usuario.")  # Mensaje en caso de interrupción manual
    except Exception as e:
        print(f"\nError: {str(e)}")  # Captura cualquier otro error inesperado
    finally:
        try:
            client.Move(0, 0, 0)  # Comando para detener el movimiento del robot
            client.StopMove()  # Detener cualquier acción en curso
            print("\nRobot detenido correctamente.")
        except Exception as e:
            print(f"\nError al detener el robot: {str(e)}")  # Mensaje en caso de fallo al detener el robot

# Punto de entrada del script
if __name__ == "__main__":
    main()
