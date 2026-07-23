# **Código en Python para Movimiento de Brazos con arm_sdk**

Para mayor información de como se creó este código se puede remitir a [Documentacion Unitree](https://support.unitree.com/home/en/G1_developer/arm_control_routine)

## **Requerimientos Previos**

✔ **Robot G1 encendido** y conectado al PC mediante ​**Ethernet**​.
✔ **PC con SDK Python de Unitree** instalado y configurado.
✔ Archivo **`g1_arm7_sdk_dds_example.py`** disponible en la ruta:

```bash
~/unitree_sdk2_python/example/g1/high_level/
```

✔ Conexión estable con el robot (​**verificable con `ping 192.168.123.161`**​).
✔ **Modo de parada bloqueada activado** (`L1 + UP`) antes de iniciar pruebas.
✔ ​**Teclado conectado**​, ya que el script requiere **entrada por teclado** para avanzar en las pruebas.

## **Acceder a la SDK en el PC**

Abrir una terminal en el PC conectado al robot y navegar hasta la carpeta del script:

```bash
cd  ~/unitree_sdk2_python/example/g1/high_level/
```

## **Ejecutar el Script de Prueba**

Ejecutar el código con el nombre de la interfaz de red que conecta el PC con el robot (por ejemplo, `enp3s0`):

```bash
python3 g1_arm7_sdk_dds_example.py enp3s0
```

## **Secuencia de Movimientos**

El programa ejecuta una secuencia de movimientos en cuatro etapas. Primero, los brazos se desplazan a la posición inicial con todas las articulaciones en cero radianes, asegurando su alineación antes de continuar. Luego, se extienden horizontalmente y permanecen en esa postura durante cinco segundos. Posteriormente, descienden lentamente, manteniendo la posición por el mismo tiempo. Finalmente, el control de alto nivel se desactiva de forma progresiva en un lapso de dos segundos para garantizar una transición segura.

