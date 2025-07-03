# **Control Manual con Teclado: API loco_client en el G1 de Unitree**

Este documento describe cómo ejecutar una prueba de movimiento manual utilizando la API `loco_client` en el robot **G1 de Unitree**, controlado mediante teclas.

## **Requisitos Previos**

**Asegurate de tener:**

1. **Robot G1 colgado en modo normal (cero torque).**
2. **PC conectado al robot vía Ethernet.**
3. **Entorno con la SDK de Python de Unitree correctamente configurado.**
4. **Archivo `g1_wasd_control.py` ubicado en:**
   ```bash
   ~/unitree_sdk2_python/example/g1/high_level/
   ```

### **Ejecución**

Una vez ubicado el código, abre la terminal y desde la carpeta high_level, ejecutar:

```bash
python3 g1_wasd_control.py nombreInterfaz
```

Remplazar nombreInterfaz con la interfaz de red correspondiente (ej. eno1, eth0).

Luego, sigue las instrucciones en la terminal:

1. Espera a que el robot se inicialice.
2. Usa el control remoto del G1 para colocarlo en **Main Operation Control**.
3. Cuando aparezca:
   
   ```bCurrent
   Current Status: Robot Ready
   ```

¡El robot está listo para recibir comandos desde el teclado!

## **Controles Disponibles**

| Tecla     | Acción                    |
| --------- | -------------------------- |
| `W`     | Avanzar                    |
| `S`     | Retroceder                 |
| `A`     | Desplazarse a la izquierda |
| `D`     | Desplazarse a la derecha   |
| `Q`     | Rotar hacia la izquierda   |
| `E`     | Rotar hacia la derecha     |
| `Space` | Detener el movimiento      |
| `Esc`   | Finalizar la ejecución    |

## **Notas de Seguridad**

* **Verifica que no haya obstáculos ni personas cerca** del robot antes de iniciar.
* El control es **directo** y **no tiene sensores de freno automático**.
* Para detener el robot de forma segura: presiona `Space` y luego `Esc`.

## **¿Qué hace el código?**

* Inicializa la conexión con el robot mediante `loco_client`.
* Solicita al robot que se **ponga de pie** y entre en modo **Balance Stand**.
* Escucha tus teclas y envía comandos de movimiento en tiempo real.

## **¿Problemas comunes?**

* ¿No pasa nada al ejecutar? ✅ Revisa que estés en la carpeta correcta y que tu interfaz de red sea válida.
* ¿El robot no se mueve? ✅ Asegúrate de haber activado el modo **Main Operation Control** desde el control remoto.
* ¿Errores en la terminal? ✅ Revisa que tengas todas las dependencias instaladas.

