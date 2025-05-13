# Movimientos Articulares Superiores con Análisis de Torque Unitree G1

**Descripción general:**
Esta prueba está diseñada para controlar y analizar el comportamiento de las articulaciones superiores del robot cuadrúpedo **G1 de Unitree**, evaluando los **torques generados** al aplicar diferentes pesos sobre los brazos del robot, tanto en estado **estático** como en **movimiento de marcha** utilizando el control remoto.

## Objetivo General

Esta herramienta dual permite:

* Controlar de forma precisa y flexible las **articulaciones superiores** del robot G1 mediante la **SDK2**.
* Visualizar en tiempo real los valores de **posición** y **torque estimado** de cada articulación.
* Ideal para pruebas, validación de movimientos programados y análisis de capacidades articulares.

## Scripts Utilizados

| Script                                  | Descripción                                                           |
| --------------------------------------- | ---------------------------------------------------------------------- |
| `g1_armsdk_moveV4.py`                 | Control interactivo de articulaciones superiores. Soporta modo manual. |
| `g1_arm_sdk_visualizer_pos_torque.py` | Visualización gráfica en tiempo real de posiciones y torques.        |

## Script 1: `g1_armsdk_moveV4.py`

### Funcionalidad

Permite controlar las articulaciones del G1 por consola.

**Modos de operación:**

* Modo manual: ingreso de posiciones articulares en radianes.

### Requisitos

* Conexión Ethernet activa (e.g. `192.168.123.X`)
* Robot en modo `Main Operation Control` y con arnés de seguridad.
* SDK2 funcional.
* Python 3, con:

```bash
pip install numpy csv pyqtgraph PyQt5
```

* Se recomienda G1 con **29 DoF**.
* Permisos de escritura para generar `.csv`.

### Ejecución

```bash
python3 g1_armsdk_moveV4.py eth0
```

### Funcionamiento

1. **Inicialización:** conexión con DDS (`lowstate`, `arm_sdk`), verificación de LocoClient.
2. **Lectura de estado inicial:** punto de partida articular.
3. **Interfaz interactiva:** ingreso de valores (Enter = 0.0 rad, `exit` = terminar).
4. **Interpolación:** movimiento suave entre posiciones (default: 5s).
5. **Liberación:** cierre de canal, guardado del `.csv`.

### Salida

Archivo: `data_g1_YYYYMMDD_HHMMSS.csv`
Columnas:
`timestamp, q0, τ0, q1, τ1, ..., qN, τN`

## Script 2: `g1_arm_sdk_visualizer_pos_torque.py`

### Funcionalidad

Visualiza en tiempo real los valores de posición (`q`) y torque estimado (`τ`) desde un archivo `.csv`.

### Requisitos

* Python 3

  ```bash
  pip install pyqtgraph PyQt5
  ```
* Archivo `.csv` generado por `g1_armsdk_moveV4.py`.

### Ejecución

```bash
python3 g1_arm_sdk_visualizer_pos_torque.py
```

Luego, ingresar ruta del `.csv`:

```bash
data_g1_20250409_183200.csv
```

### Funcionamiento

* Ventana con dos gráficos:
  * Posición: línea continua.
  * Torque: línea punteada.
* Checkboxes para seleccionar articulaciones.
* Actualización automática cada 50ms (\~20Hz).
* Zoom sincronizado.

## Uso en Simultáneo

1. Terminal 1 (control del robot):

   ```bash
   python3 g1_armsdk_moveV4.py eth0
   ```
2. Terminal 2 (visualizador):

   ```bash
   python3 g1_arm_sdk_visualizer_pos_torque.py
   ```

Introduce el mismo archivo `.csv` que se está generando.
Requiere permisos de **lectura concurrente**.

## Protocolo de Evaluación

1. Colocar diferentes **pesos en los brazos** del robot.
2. Ejecutar movimientos **estáticos** y luego **dinámicos** (caminata con control remoto).
3. Evaluar el comportamiento del torque en función del peso y movimiento.
