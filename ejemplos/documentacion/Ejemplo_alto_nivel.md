# **Ejemplo de Alto Nivel - G1**

Este ejemplo demuestra cómo controlar la locomoción del **G1** de Unitree utilizando **loco\_client** en **C++** y **Python**.

## **Importante:**

* **NO** activar **Debug Mode**, ya que esto deshabilita el control de operación incorporado y anula el servicio de locomoción de alto nivel. Si se encuentra activado se debe apagar y volver a encender el robot.
* Se recomienda consultar la [documentación oficial](https://support.unitree.com/home/en/G1_developer/rpc_routine) para conocer todos los comandos disponibles y su configuración detallada en C++.

## **Control de Locomoción de Alto Nivel con loco\_client en C++ y Python**

A continuación, se presenta una introducción a los comandos básicos disponibles en **C++ y Python** para el control del G1 mediante **loco\_client**, junto con una secuencia de prueba recomendada para validar la transición entre modos.

### **Ejecución en C++**

Para ejecutar comandos en **C++**, primero asegúrate de haber seguido los pasos de encendido estándar del robot que se encuentre en el modo Main Operation Control y conectado correctamente. Luego, **SIN activar Debug Mode**, ejecuta:

```bash
./g1_loco_client --network_interface=enp3s0 --set_velocity="0.5 0 0 1"
```

Esto hará que el robot se mueva hacia adelante a una velocidad de 0.5 m/s por 1 segundo.

De manera general, los parámetros en **C++** siguen la sintaxis:

```bash
--${key}=${value}   # Si el parámetro requiere un valor
--"${key}"          # Si el parámetro no requiere valor
```

Algunos de los parámetros clave disponibles en **loco\_client** son los siguientes:

### **Comandos con valores**


| **Comando**                   | **Descripción**                                                  |
| ----------------------------- | ----------------------------------------------------------------- |
| `move="vx, vy, vyaw"`         | Mueve el robot con velocidad (vx, vy, vyaw) durante**1 segundo**. |
| `set_velocity="vx, vy, vyaw"` | Configura la velocidad del robot de manera continua.              |
| `set_fsm_id="ID_modo"`        | Permite cambiar entre modos de locomoción según su ID.          |

### **Comandos sin valores**


| **Comando**       | **Descripción**                                 |
| ----------------- | ------------------------------------------------ |
| `--damp`          | Activa el**modo de suspensión (damping mode)**. |
| `--start`         | Activa el**control de movimiento principal**.    |
| `--squat`         | Modo**Squat**.                                   |
| `--sit`           | Modo**sentado**.                                 |
| `--stand_up`      | Modo**de pie**.                                  |
| `--zero_torque`   | Activa el**modo sin torque**.                    |
| `--stop_move`     | **Detiene el movimiento**del robot.              |
| `--high_stand`    | Ajusta la**altura máxima de marcha**.           |
| `--low_stand`     | Ajusta la**altura mínima de marcha**.           |
| `--balance_stand` | Activa el**modo de equilibrio dinámico**.       |

### **Ejecución en Python**

Para ejecutar comandos en **Python**, primero asegúrate de estar en la ruta donde se encuentra el archivo (generalmente *\~/unitree\_sdk2\_python/example/g1/high\_level*) y de que la librería de loco\_client esté correctamente instalada. Luego, ejecuta:

```bash
python3 g1_loco_client_example.py networkInterface
```

Al ejecutar el script, se desplegará una lista de opciones. Dependiendo de la versión de la **SDK2** instalada, los comandos disponibles pueden variar.

#### **Versiones de SDK2 y opciones disponibles**

**Si usas una versión anterior a marzo de 2025:**

```python
option_list = [
    TestOption(name="damp", id=0),       
    TestOption(name="stand_up", id=1),   
    TestOption(name="sit", id=2),   
    TestOption(name="move forward", id=3),       
    TestOption(name="move lateral", id=4),  
    TestOption(name="move rotate", id=5),  
    TestOption(name="low stand", id=6),  
    TestOption(name="high stand", id=7),  
    TestOption(name="zero torque", id=8),
    TestOption(name="wave hand1", id=9),  
    TestOption(name="wave hand2", id=10),  
    TestOption(name="shake hand", id=11),   
]
```

**Si usas la versión más reciente (marzo 2025):**

```python
option_list = [
    TestOption(name="damp", id=0),       
    TestOption(name="Squat2StandUp", id=1),   
    TestOption(name="StandUp2Squat", id=2),   
    TestOption(name="move forward", id=3),       
    TestOption(name="move lateral", id=4),  
    TestOption(name="move rotate", id=5),  
    TestOption(name="low stand", id=6),  
    TestOption(name="high stand", id=7),  
    TestOption(name="zero torque", id=8),
    TestOption(name="wave hand1", id=9),  
    TestOption(name="wave hand2", id=10),  
    TestOption(name="shake hand", id=11),   
    TestOption(name="Lie2StandUp", id=12),    
]

```

# **Protocolo de Prueba – Secuencia de Control de Locomoción en loco\_client C++**

### **Objetivo**

Validar la correcta transición entre modos de locomoción del **G1 de Unitree** usando **loco\_client**, asegurando el funcionamiento esperado.

### **Requerimientos**

**Robot G1** encendido y conectado a la red.
**PC con loco\_client** correctamente instalado y configurado.
**Conexión de red estable** (verificar con `ping 192.168.123.161`).

### **Procedimiento**

#### **Encendido y conexión del robot**

1. Encender el robot siguiendo el **procedimiento estándar**.
2. **NO ingresar al Debug Mode** (esto deshabilita el control de operación).
3. Verificar la conexión de red con:
   ```bash
   ping 192.168.123.161
   ```

#### **Ejecución de la secuencia de modos**

Ejecutar los siguientes comandos en **C++**,  validando el comportamiento del robot tras cada uno (recuerda cambiar el nombre de la interfaz por el propio):

```bash
./g1_loco_client --network_interface=enp3s0  --damp
./g1_loco_client --network_interface=enp3s0  --stand_up
./g1_loco_client --network_interface=enp3s0  --start
./g1_loco_client --network_interface=enp3s0  --move="0.3 0 0"
./g1_loco_client --network_interface=enp3s0  --damp
./g1_loco_client --network_interface=enp3s0  --zero_torque
```

### **Resultados Esperados**


| **Modo**             | **Comportamiento esperado**                                                   |
| -------------------- | ----------------------------------------------------------------------------- |
| **Damping Mode**     | El robot entra en suspensión con rigidez mínima.                            |
| **Stand Up**         | El robot se levanta a su posición inicial, posición get ready o lock stand. |
| **Start**            | Se activa el control de movimiento principal.                                 |
| **Move (0.3, 0, 0)** | El robot avanza en línea recta a 0.3 m/s por 1 segundo.                      |
| **Damping Mode**     | El robot vuelve a estado de suspensión.                                      |
| **Zero Torque**      | Se desactiva el torque de los motores, permitiendo movimiento libre.          |

### **Otros modos disponibles para prueba**

Además de los comandos anteriores, **loco\_client** permite acceder a otros modos que pueden ser evaluados en pruebas adicionales considerando las secuencias de modos de la [documentacion oficial](https://support.unitree.com/home/en/G1_developer/remote_control):


| **Comando**       | **Descripción**                               |
| ----------------- | ---------------------------------------------- |
| `--squat`         | Modo**Squat**.                                 |
| `--sit`           | Modo **sentado**.                             |
| `--stop_move`     | **Detiene**cualquier movimiento en ejecución. |
| `--high_stand`    | Configura la**altura máxima de marcha**.      |
| `--low_stand`     | Configura la**altura mínima de marcha**.      |
| `--balance_stand` | Activa el**modo de equilibrio dinámico**.     |

📌 **Para más información sobre las opciones avanzadas, revisa la documentación oficial de loco\_client.**
