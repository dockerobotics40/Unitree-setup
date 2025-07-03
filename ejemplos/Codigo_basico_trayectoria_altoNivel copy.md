# **Movimiento Automático Predefinido: API `loco_client` en el G1 de Unitree**

Este documento describe cómo ejecutar una secuencia automática de movimientos utilizando la API `loco_client` en el robot **G1 de Unitree**, como parte de un reto de locomoción básica.

## **Requisitos Previos**

1. Robot **G1 en el suelo**, con espacio libre para desplazarse.
2. **PC conectado al robot vía Ethernet.**
3. SDK de Python de Unitree correctamente configurada.
4. Archivo `g1_moveInTime.py` ubicado en:

   ```bash
   ~/unitree_sdk2_python/example/g1/high_level/
   ```

### **Precaución**

🔴 El robot realizará movimientos automáticos en forma de cuadrado y un gesto de saludo.
Antes de ejecutar, asegúrate de:

- Estar en una superficie plana, estable y sin obstáculos.

- Tener el robot encendido y en modo normal (cero torque al inicio).

- Activar el modo de operación (R1 + X) desde el control remoto cuando el script lo indique.

### Ejecución

Desde la carpeta high_level, ejecutar:

```echarts
python3 g1_moveInTime.py nombreInterfaz
```


📌 Reemplaza nombreInterfaz por el nombre de la interfaz de red correspondiente (ej. eno1, eth0).

El script pedirá confirmaciones antes de iniciar la secuencia. Luego realizará automáticamente:

- Cuatro desplazamientos con giros de 90° simulando un cuadrado.

- Un gesto de saludo final con la "mano" del robot.

- Detención segura del robot.

## **¿Qué hace exactamente el script?** 

- Se conecta al robot y lo pone de pie con la API loco_client.

- Ejecuta 4 movimientos hacia adelante y 4 giros para trazar un cuadrado.

- Luego realiza un gesto de saludo usando client.WaveHand().

- Finalmente, detiene cualquier movimiento activo.

## **Recomendaciones**

- Si el robot no se mueve, asegúrate de activar el control remoto en modo operación (R1 + X).

- Mantén siempre un área libre alrededor del robot durante la prueba.

- Para detener la ejecución antes de tiempo, presiona Ctrl+C en la terminal.

## **¿Problemas comunes?**

El robot no responde → Verifica la conexión Ethernet, que la interfaz de red esté bien escrita y que esté en modo operación.

No se detiene → Usa Ctrl+C y asegúrate de que se haya ejecutado client.StopMove() en el finally.

¡Y listo! Tu G1 ejecutará un pequeño reto autónomo para demostrar sus capacidades de locomoción y gesticulación
