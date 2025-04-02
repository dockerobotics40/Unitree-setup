# Conexión del Robot G1: Configuración de Red

## **Pasos Iniciales**

Antes de utilizar los recursos del robot G1, sigue estos pasos para establecer la conexión correctamente:

1. **Conectar el robot** al PC mediante un **cable Ethernet**.
2. **Encender el robot** desde el soporte de alimentación.
3. **Esperar a que el robot inicie** y quede en estado suspendido en el soporte.

## **Activar Debug Mode en el Control Remoto**

Para habilitar el modo de depuración en el robot:

1. Presiona **L2 + R2** en el control remoto para entrar en *Debug Mode*.
2. Luego, presiona **L2 + A** para confirmar que el robot está en este modo.
   (El robot se posicionará en **cero** en todas sus articulaciones).
3. Finalmente, activa el **Damping Mode** presionando **L2 + B**.

## **Configuración de Red en el PC**

Ahora, configura la dirección IP en tu computador:

1. **Abrir la configuración de red:**

- Ve a **Settings > Network**
- Asigna la dirección **192.168.123.222** a tu PC.
- El robot tiene la IP **192.168.123.161**.
  IMAGENES

2. **Validar la conexión** con los siguientes comandos en la terminal:

Se realiza el ping en el terminal

```bash
ping 192.168.123.161
```

IMAGENES
Luego comprobamos el nombre de la conexión usando en la terminal

```bash
ifconfig
```

IMAGEN

Esto te mostrará las interfaces de red disponibles y te ayudará a identificar el nombre de la conexión activa con G1.

---

**Nota:** Es importante asegurarse de que la dirección IP del PC esté dentro del mismo rango que el robot para que la comunicación sea exitosa.

Ahora estás listo para interactuar con el **G1** desde tu PC con los diferentes ejemplos.

