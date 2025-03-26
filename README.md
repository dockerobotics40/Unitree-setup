# Robotics 4.0 - Documentación y Recursos para Unitree  

Bienvenido al repositorio de **Robotics 4.0**, un equipo dedicado a la investigación, desarrollo y aplicación de soluciones robóticas avanzadas. Nuestro objetivo es facilitar la implementación de tecnologías en robótica mediante documentación estructurada y ejemplos prácticos.  

Este repositorio es una **condensación de todos los recursos disponibles** sobre la instalación y uso de repositorios de **Unitree**, además de incluir **códigos básicos creados por nuestro equipo** para facilitar el aprendizaje y la experimentación.  

---

## 📁 Estructura del Repositorio  

```bash
📦 unitree-setup  
│-- 📄 README.md               # Documentación principal  
│-- 📂 docs/          # Guías de instalación de SDKs y paquetes  
│   ├── Instalacion_unitree_sdk2.md        # Instalación y uso de SDK2 de Unitree  
│   ├── Instalacion_unitree_sdk2_python.md       # Instalación de otros paquetes
│   ├── Instalacion_unitree_ros2.md       # Instalación de otros paquetes
│   ├── Instalacion_driver_LiDAR_LIVOX360.md       # Instalación de otros paquetes
│   ├── Instalacion_driver_depth_camera.md       # Instalación de otros paquetes   
│-- 📂 ejemplos/               # Ejemplos de uso básicos  
│   ├── Conexion_robotG1.md            # Publicador-suscriptor  
│   ├── Ejemplo_bajo_nivel.md            # Explicado para python y C++ SDK
│   ├── Ejemplo_alto_nivel.md            # Explicado para python y C++ SDK
│   ├── Ejemplo_movimiento_brazos.md            # Explicado para python y C++ arm_sdk
│   ├── Ejemplo_acceso_sensores_ROS2.md            # Explicado visualización ROS2
│   ├── Codigo_basico_movimientos_altoNivel.md            # Explicado wasd y trayectoria personalizada*
│   ├── Codigo_basico_movimientoBrazo_personalizado.md            # Explicado arm_sdk poner articulaciones en terminal
│   ├── Visualizacion_info_LiDAR.md            # Explicado proceso
│   ├── Visualizacion_info_camaraDepth.md            # Explicado proceso
│   ├── Conexion_wifi_PC2.md            # Explicado como se hace y que es PC2
│-- 📂 repositirios/               # Enlaces de documentación original
│   ├── Enlaces_repositorios.md            # Direcciones originales  
│-- 📂 imagenes/                 # Capturas de pantalla y diagramas  
```
## 📌 Contenidos
### 1. Instalaciones
Aquí encontrarás guías detalladas para instalar:

- SDK2 de Unitree para python y C++
- Unitree_ROS2 para manejo de G1 desde ROS2
- Drivers para uso del LiDAR y cámara integrados

  
📍 Para empezar, consulta la guía de instalación del SDK2 de Unitree en [docs/Instalacion_unitree_SDK2.md](docs/Instalacion_unitree_sdk2.md).

### 2. Ejemplos Básicos y personalizables
Una vez instalado el SDK2 en python/C++, paquete de ROS2 para Unitree, drivers y configurado el entorno, puedes probar ejemplos como:

- Control de bajo nivel del robot (movimiento de sus tobillos).
- Control de alto nivel usando la API de unitree.
- Control de movimientos personaliados de los brazos.
- Visualización de sensores exteroceptivos.
- Conexión WiFi desde el computador de desarrollo del robot.


📚 Puedes encontrar los ejemplos en la carpeta examples/.



🤝 Conocer al Equipo
Somos Robotics 4.0, un equipo de entusiastas de la robótica que busca innovar y desarrollar herramientas accesibles para la comunidad.

📌 (Aquí se puede agregar una breve descripción del equipo y sus miembros con enlaces a perfiles o proyectos destacados).


📜 Licencia
Este proyecto sigue la licencia MIT.




