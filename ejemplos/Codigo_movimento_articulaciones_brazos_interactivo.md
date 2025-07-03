# Control de Articulaciones Superiores del Unitree G1 + Registro de Torque

## Descripción general

Este módulo permite controlar las **articulaciones superiores** del robot cuadrúpedo **G1 de Unitree** y registrar el comportamiento de **posición y torque estimado** en tiempo real. Es ideal para pruebas de carga, validación de movimientos y análisis biomecánico en robótica.

## Objetivo General

Este sistema permite:

- Controlar de forma **manual e interactiva** las articulaciones del G1 mediante la SDK2.
- Visualizar los valores de posición (`q`) y torque estimado (`τ`) en un archivo `.csv`.-
- Probar el impacto de **cargas externas** o movimientos sobre el torque de cada articulación.

## Scripts Utilizados

| Script                                | Descripción                                                                |
| ------------------------------------- | --------------------------------------------------------------------------- |
| `g1_armsdk_moveV4.py`                 | Control manual de las articulaciones superiores del G1 vía consola.        |
| `g1_arm_sdk_visualizer_pos_torque.py` | Visualizador gráfico en tiempo real de posiciones y torques desde un`.csv` |

## Script Principal: `g1_armsdk_moveV4.py`

### Funcionalidad

Controlar manualmente las articulaciones del robot G1 a través de una interfaz por consola.
Registra automáticamente la posición y torque de cada articulación en un archivo `.csv`.

### Requisitos

* Conexión Ethernet activa con el robot (ej. `eth0`, IP en `192.168.123.X`)
* Robot G1 en **modo de operación normal** (`Main Operation Control`)
* SDK2 instalada y funcional
* Python ≥ 3.6
* Instalar dependencias:

```bash
pip install numpy csv pyqtgraph PyQt5
```

* Robot G1 con al menos **29 DoF** (para brazos y torso)
* Permisos de escritura para crear archivos `.csv`

### Ejecución

Desde la carpeta donde ubicaste el código:

```bash
python3 g1_armsdk_moveV4.py nombreInterfaz
```

### Funcionamiento paso a paso

1. **Conexión inicial:**
   * Se conectan los canales `lowstate` y `arm_sdk` vía DDS.
   * Se espera el primer mensaje de estado (`LowState`).
2. **Posición inicial:**
   * El robot se mueve automáticamente a **posición cero** (`q = 0.0`) para comenzar.
   * Opción: modificar esta lógica para usar una **posición inicial de descanso**.
3. **Interfaz manual (en consola):**
   * El usuario ingresa manualmente los valores deseados para cada articulación (en radianes).
   * Presionar **Enter** sin escribir usa `0.0`.
   * Escribir `exit` cancela el ingreso actual.
4. **Movimiento interpolado:**
   * El robot realiza un movimiento suave hacia la nueva posición objetivo (duración por defecto: 5 segundos).
   * Se verifica si se alcanzó la posición deseada con una tolerancia de 0.05 rad.
5. **Registro en `.csv`:**
   * Cada 500 ciclos de control (\~10s), se guarda en un `.csv` el timestamp, las posiciones `q` y torques `τ` actuales de cada articulación.
6. **Liberación segura del control:**
   * Al salir, el robot se mueve a una **posición de descanso predefinida**.
   * Luego se desactiva `arm_sdk` y se cierra el archivo `.csv`.

### Salida generada

* Archivo: `data_g1_YYYYMMDD_HHMMSS.csv`
* Contenido:
  ```bash
  timestamp, q_joint15, tau_joint15, ..., q_joint28, tau_joint28
  ```

## Visualizador en Tiempo Real: `g1_arm_sdk_visualizer_pos_torque.py`

### Funcionalidad

Visualiza dinámicamente los valores registrados en el `.csv` generado por el script anterior. Ideal para observar patrones de torque en pruebas con peso o movimiento.

### Ejecución

```bash
python3 g1_arm_sdk_visualizer_pos_torque.py
```

Se abrirá una ventana gráfica. Ingresa la ruta del archivo `.csv` generado.


### Visualización

* Gráfica 1: posición articular (`q`) – línea continua
* Gráfica 2: torque estimado (`τ`) – línea punteada
* Controles:
  * Checkboxes para seleccionar articulaciones
  * Zoom sincronizado en ambos gráficos
  * Actualización automática cada \~50ms (20Hz)

## Uso en paralelo

1. En una terminal (para el robot):
   
   ```bash
   python3 g1_armsdk_moveV4.py nombreInterfaz
   ```
2. En otra terminal (para visualización):
   
   ```bash
   python3 g1_arm_sdk_visualizer_pos_torque.py
   ```

Asegúrate de que ambos acceden al mismo archivo `.csv`. El visualizador requiere que el archivo esté siendo actualizado en tiempo real (lectura concurrente).




