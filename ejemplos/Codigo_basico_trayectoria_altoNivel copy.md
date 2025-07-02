# **Movimiento Automático Predefinido: API loco_client en el G1 de Unitree**


Este documento describe cómo ejecutar una prueba de movimiento automático utilizando la API `loco_client` en el robot **G1 de Unitree**, siguiendo una trayectoria programada.

## **Requisitos Previos**

1. **Robot G1 en el suelo y en un entorno seguro para moverse.**
2. **PC conectado al robot vía Ethernet.**
3. **Entorno con la SDK de Python de Unitree correctamente configurado.**
4. **Archivo `g1_moveInTime.py` ubicado en:**
   ```bash
   ~/unitree_sdk2_python/example/g1/high_level/
   ```

### **⚠️ Precaución**

🔴 El robot se moverá de forma automática para ejecutar un cuadrado. Asegúrate de que esté en una superficie plana, estable y sin obstáculos antes de iniciar.

## **Ejecución**

Desde la carpeta high_level, ejecutar:

```bash
python3 g1_moveInTime.py nombreInterfaz
```

Reemplazar nombreInterfaz con la interfaz de red correspondiente.


Al iniciar, el programa pedirá inicializar el estado de main operation control del robot con el control remoto. Una vez listo, comenzará a ejecutar un patrón cuadrado de movimiento automáticamente.

### **Notas Adicionales**

- Verifica la conectividad con el robot antes de iniciar.
- Asegura que el área esté despejada.
- Para detener la ejecución, presiona Esc o interrumpe el proceso en la terminal.


