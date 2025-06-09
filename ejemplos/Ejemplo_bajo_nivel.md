# **Ejemplo de Bajo Nivel - G1**

Este ejemplo demuestra cómo ejecutar una rutina de **movimiento de tobillos** en el robot **G1** de Unitree.

## **Requisitos previos**

🔹 Tener la **SDK2 de Unitree** instalada y configurada.
🔹 Conexión de red configurada con el robot.
🔹 El **G1** debe estar colgado para evitar caídas inesperadas.
🔹 Modo **Debug Mode** activado.

## **Ejecución en C++**

### **Pasos para ejecutar el ejemplo en C++:**

1. **Abrir una terminal y dirigirse a la SDK:**

```bash
cd
cd unitree_sdk2
```

2. **Ejecutar el ejemplo de movimiento de tobillos:**

```bash
./build/bin/g1_ankle_swing_example
```

📌 **Nota:**`nombreInterfaz` debe reemplazarse con el nombre de la interfaz de red del computador encontrada en [Conexion_robotG1](Conexion_robotG1.md)

### **¿Qué hace este código?**

* Mueve todas las articulaciones del robot a la **posición cero**.
* Balancea sus tobillos en modalidad **PR**.
* Balancea sus tobillos en modalidad **AB**.
  Para mayor información se puede consultar en la [Documentacion Unitree](https://support.unitree.com/home/en/G1_developer/basic_motion_routine).

## **Ejecución en Python**

### **Pasos para ejecutar el ejemplo en Python:**

1. **Abrir una terminal y dirigirse a la carpeta del ejemplo:**
   
   ```bash
   cd
   cd unitree_sdk2_python/example/g1/low_level
   ```
2. **Ejecutar el script:**
   
   ```bash
   python3 g1_low_level_example.py nombreInterfaz
   ```

📌 **Nota:** Reemplazar `nombreInterfaz` con el nombre de la interfaz de red, obtenida.

### **¿Qué hace este código?**

* Restablece el robot a la **posición cero** desde cualquier estado inicial.
* Ejecuta un balanceo de los **tobillos** en dos modos diferentes.
* Imprime los **ángulos de Euler** con una determinada frecuencia.

📍 **Ubicación del código fuente:**

* **C++:**`unitree_sdk2/example/g1/low_level/g1_ankle_swing_example.cpp`
* **Python:**`unitree_sdk2_python/example/g1/low_level/g1_low_level_example.py`

