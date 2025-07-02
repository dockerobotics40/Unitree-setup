# **Control Manual con Teclado: API loco_client en el G1 de Unitree**

Este documento describe cómo ejecutar una prueba de movimiento manual utilizando la API `loco_client` en el robot **G1 de Unitree**, controlado mediante teclas.

## **Requisitos Previos**

1. **Robot G1 colgado en modo normal (cero torque).**
2. **PC conectado al robot vía Ethernet.**
3. **Entorno con la SDK de Python de Unitree correctamente configurado.**
4. **Archivo `g1_wasd_control.py` ubicado en:**
   ```bash
   ~/unitree_sdk2_python/example/g1/high_level/
   ```

### **Ejecución**

Una vez ubicado el código, desde la carpeta high_level, ejecutar:

```bash
python3 g1_wasd_control.py nombreInterfaz
```

Remplazar nombreInterfaz con la interfaz de red correspondiente (ej. eno1, eth0).

Al iniciar, el programa pedirá inicializar el estado de main operation control del robot usando el control remoto. Luego, cuando aparezca en la terminal:

```bash
Current status: Robot Ready
```

Significa que el robot está listo para recibir comandos de teclado.

### **Controles Disponibles**

| Tecla   | Acción                    |
| ------- | -------------------------- |
| `W`     | Avanzar                    |
| `S`     | Retroceder                 |
| `A`     | Desplazarse a la izquierda |
| `D`     | Desplazarse a la derecha   |
| `Q`     | Rotar hacia la izquierda   |
| `E`     | Rotar hacia la derecha     |
| `Space` | Detener el movimiento      |
| `Esc`   | Finalizar la ejecución    |

### **Notas de Seguridad**

Verifica que el entorno sea seguro antes de enviar comandos.

El robot debe estar colgado/ sostenido con grúa para evitar caídas accidentales.

