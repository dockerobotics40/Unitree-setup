# **Codigos básicos de movimiento con la API loco\_client en el G1 de Unitree**

Este documento describe los pasos para ejecutar pruebas de movimiento utilizando la API `loco_client` en el robot ​**G1 de Unitree**​.

## **Requisitos Previos**

1. **Robot G1 colgado en modo normal (cero torque).**
2. **PC conectado al robot vía Ethernet.**
3. **Entorno con la SDK de Python de Unitree correctamente configurado.**
4. **Archivos de prueba ubicados en la ruta recomendada:**
   ```bash
   ~/unitree_sdk2_python/example/g1/high_level/
   ```
## **Prueba 1: Control Manual con WASD (`g1_wasd_control.py`)**

Esta prueba permite controlar manualmente el movimiento del robot utilizando el teclado.

### **Ejecución**

Desde la carpeta `high_level`, ejecutar el siguiente comando:

```bash
python3 g1_wasd_control.py nombreInterfaz
```

Reemplazar `nombreInterfaz` con la interfaz de red correspondiente.

### **Controles Disponibles**

| Tecla       | Acción                    |
| ------------- | ---------------------------- |
| `W`     | Avanzar                    |
| `S`     | Retroceder                 |
| `A`     | Desplazarse a la izquierda |
| `D`     | Desplazarse a la derecha   |
| `Q`     | Rotar hacia la izquierda   |
| `E`     | Rotar hacia la derecha     |
| `Space` | Detener el movimiento      |
| `Esc`   | Finalizar la ejecución    |

Al iniciar, el programa muestra:

```bash
Current status: Robot Ready
```


Indicando que está listo para recibir comandos.

## **Prueba 2: Movimiento Automático con `g1_moveInTime.py`**

Esta prueba ejecuta un patrón de movimiento automático donde el robot se desplazará en un **cuadrado** sin intervención manual.

### ⚠️ **Precaución**

🔴 **El robot se moverá de forma automática.** Asegúrate de que esté colgado de manera segura antes de ejecutar esta prueba.

### **Ejecución**

Desde la carpeta `high_level`, ejecutar:

```bash
python3 g1_moveInTime.py nombreInterfaz
```

Reemplazar `nombreInterfaz` con la interfaz de red correspondiente.

## **Notas Adicionales**

* **Verifica la conectividad con el robot** antes de iniciar las pruebas.
* **Asegura que el robot esté en una posición segura** antes de realizar movimientos.
* **Para detener la ejecución en cualquier momento,** usa `Esc` o interrumpe el proceso en la terminal.

