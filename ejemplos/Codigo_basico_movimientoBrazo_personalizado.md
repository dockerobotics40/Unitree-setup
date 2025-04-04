# **Movimientos Articulados Superiores con Ángulos Más Complejos**

## **Objetivo**

Verificar la ejecución de movimientos articulados superiores del G1 de Unitree con ángulos más complejos, evaluando la precisión y estabilidad del control.

## **Requerimientos Previos**

✔ Robot **G1 encendido** y conectado a la red mediante **Ethernet**.

✔ PC con **SDK Python** de Unitree instalada y correctamente configurada.

✔ Archivo **`g1_arm_sdk_moveV2.py`** disponible en la ruta `~/unitree_sdk2_python/example/g1/high_level/`.

✔ Conexión estable con el robot (verificable con `ping 192.168.123.161`).

✔ Robot **suspendido** y en **modo de parada bloqueada (L1 + UP)** para realizar pruebas de forma segura.

## **Acceder a la SDK en el PC**

En la terminal del PC conectado al G1, navegar hasta la carpeta donde está el script:

```bash
cd ~/unitree_sdk2_python/example/g1/high_level/
```

## **Ejecutar el Script de Movimiento**

Correr el código de control del brazo, especificando la interfaz de red utilizada para la conexión (por ejemplo, `enp3s0`),  el script permite ingresar comandos personalizados en la terminal para mover articulaciones específicas.

```bash
python3 g1_arm_sdk_moveV2.py enp3s0
```

Este script controla los movimientos del robot a nivel de articulaciones, interpolando posiciones suavemente y enviando comandos de bajo nivel.

## **Pruebas de Movimientos Articulados**

Para cada prueba, ingresar en la terminal los valores indicados en la columna respectiva, dejando las demás articulaciones en **0**.

### **Movimiento de muñeca (Roll, Pitch, Yaw)**

| Articulación | Izquierda (L) | Derecha (R) |
| ------------- | ------------- | ----------- |
| WRIST\_ROLL   | -0.87 rad     | 0.87 rad    |
| WRIST\_PITCH  | 0.79 rad      | -0.79 rad   |
| WRIST\_YAW    | -0.70 rad     | 0.70 rad    |

### **Movimiento de codo**

| Articulación | Izquierda (L) | Derecha (R) |
| ------------- | ------------- | ----------- |
| ELBOW         | -0.52 rad     | 1.57 rad    |

### **Movimiento combinado de codo y muñeca (brazo izquierdo)**

| Articulación | Izquierda (L) |
| ------------- | ------------- |
| ELBOW         | 1.05 rad      |
| WRIST\_PITCH  | 0.52 rad      |

### **Movimiento combinado de codo y muñeca (Brazo derecho)**

| Articulación | Derecha (R) |
| ------------- | ----------- |
| ELBOW         | 1.05 rad    |
| WRIST\_PITCH  | 0.52 rad    |

NOTA: **PRECAUCIÓN NO SUPERAR LO LÍMITES ARTICULARES DEL ROBOT**

## **Resolución de Problemas**

Si el robot **no responde correctamente**, revisar:

* Conexión Ethernet (`ping 192.168.123.161`).
* Modo de parada bloqueada activado (`L1 + UP`).
* Archivo `g1_arm_sdk_moveV2.py` ejecutándose desde la ruta correcta.
* Revisar la salida en la terminal para mensajes de error.

Si persisten problemas, intentar con el script `g1_arm_sdk_move.py` (sin threads) y verificar la comunicación con `unitree_sdk2py`.

## **Finalización del programa**

1. Llevar todas las articulaciones a la posición **0**.
2. Detener la ejecución de `g1_arm_sdk_moveV2.py` con `CTRL + C`.
3. Apagar el robot si no se requiere más pruebas.

