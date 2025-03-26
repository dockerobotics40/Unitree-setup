# Instalación de unitree_ros2 📌 
## Requerimientos
Antes de comenzar, asegúrate de contar con los siguientes requisitos en tu sistema:

- Ubuntu 20.04
- ROS2 Foxy, si no se encuentra instalado seguir: [Instalacion ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-
Debians.html)

## 1.Clonar el paquete unitree_ros2
Para comenzar, clona el repositorio necesario para interactuar con el robot desde ROS 2:
```bash
git clone https://github.com/unitreerobotics/unitree_ros2
```
## 2.Instalación de dependencias
Ejecuta los siguientes comandos para instalar las dependencias necesarias:
```bash
sudo apt update
sudo apt install ros-foxy-rmw-cyclonedds-cpp
sudo apt install ros-foxy-rosidl-generator-dds-idl
sudo apt install gedit
```
## 3.Configuración antes de compilar cyclonedds
Antes de compilar el paquete de cyclonedds, es importante verificar que ROS 2 no se inicializa automáticamente en la terminal.

### Abrir el script de inicio del terminal:
```bash
gedit ~/.bashrc
```
### Si encuentras una línea que inicializa ROS 2, coméntala (agregando # al inicio de la línea).
IMAGEN
### Guarda y cierra el archivo.
### Abre una nueva terminal y verifica que ROS 2 no se inicializa automáticamente.
IMAGEN

## 4.Compilación y clonación de paquetes de comunicación en ROS 2
Ejecuta los siguientes comandos:
```bash
cd ~/unitree_ros2/cyclonedds_ws/src
git clone https://github.com/ros2/rmw_cyclonedds -b foxy
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd ..
colcon build --packages-select cyclonedds
```
## 5.Compilación de los paquetes principales
Después de completar los pasos anteriores, compila los paquetes principales que contienen las estructuras necesarias para manejar el robot de Unitree:
```bash
source /opt/ros/foxy/setup.bash
colcon build
```
## 6.Restaurar configuración de ROS 2
Para finalizar, es necesario volver a activar la inicialización automática de ROS 2 en la terminal:

### Abrir el script de inicio:
```bash
gedit ~/.bashrc
```
### Descomentar la línea donde se inicializa ROS 2 (eliminar #).
IMAGEN
Finalmente se guarda y cerrar el archivo.

Para visualizar la referencia original [Unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)
