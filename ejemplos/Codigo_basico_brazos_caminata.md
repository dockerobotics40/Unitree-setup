# **Control Interactivo de Brazos y Locomoción del G1 (Unitree SDK2)**

## Propósito

Este script permite controlar las **articulaciones superiores** del robot G1 de Unitree de forma **manual e interactiva**, y combinarlo con **comandos de locomoción parametrizados** mediante la API `loco_client`. Es ideal para casos como:

* Pruebas de manipulación (ej. levantar una caja)
* Validación de posturas articulares
* Coordinación brazo-caminar
* Análisis de torque frente a carga y movimiento

## Requisitos

* G1 en modo `<span>Main Operation Control</span>` y conectado vía Ethernet
* PC con Python ≥ 3.6 y dependencias:

```
pip install numpy pyqtgraph PyQt5
```

* SDK2 correctamente instalada
* Robot G1 con al menos 29 DoF (para brazos, torso y muñecas)

## Ejecución

```
python3 g1_armsdk_moveV5.py <interfazRed>
```

Ejemplo:

```
python3 g1_armsdk_moveV5.py eth0
```

---

## Funcionamiento Paso a Paso

### 1. Inicialización

* Se conectan los canales DDS (`lowstate`, `arm_sdk`) y el `LocoClient`
* Se extrae el estado inicial de las articulaciones del robot

### 2. Posición Inicial

* El robot se mueve automáticamente a la **posición cero** (`q = 0.0`) al inicio
* Se solicita confirmación antes de continuar.

### 3. Control de Brazos

* Ingreso manual en consola:
  * Se solicitan los valores en radianes de cada articulación
  * Presionar **Enter** usa 0.0 por defecto
  * Escribir `exit` cancela el ingreso actual
* El robot se mueve suavemente a la postura deseada usando interpolación cosenoidal

### 4. Caminata Parametrizada

* Después de cada postura, se pregunta si se desea ejecutar una caminata
* Opciones:
  * Ingreso manual paso a paso:
    * Dirección: `adelante`, `atrás`, `izquierda`, `derecha`, `rotar_izq`, `rotar_der`
    * Tiempo en segundos
* Cada movimiento es ejecutado con confirmación y pausa

### 5. Repetición o Finalización

Después de cada secuencia:

* Opción de volver a posición cero
* Ingresar una nueva postura
* Finalizar el programa

Al salir:

* El robot se mueve automáticamente a una **posición de descanso** predefinida
* Se liberan los canales DDS y se cierra el `<span>.csv</span>`

## Salida Generada

Se guarda un archivo CSV en cada ejecución:

```
data_g1_YYYYMMDD_HHMMSS.csv
```

Columnas:

```
timestamp, q_joint15, tau_joint15, ..., q_joint28, tau_joint28
```

Frecuencia de guardado: cada 500 ciclos de control (aprox. 10s)

## Uso Típico en Escenarios Experimentales

1. **Definir posturas** desde simulador (ej. MuJoCo) o pruebas manuales
2. **Registrar** esas posturas en código como diccionarios
3. **Definir trayectorias** en estructuras tipo lista para caminata
4. Ejecutar cada fase:
   * Aproximación
   * Agarre
   * Caminata con carga
   * Liberación del objeto

## Notas Técnicas

* Movimiento suave entre posturas gracias a interpolación cosenoidal
* Validación de posición alcanzada con tolerancia configurable (`<span>0.05</span>` rad por defecto)
* Modularidad para integrar nuevas posturas o trayectorias fácilmente


