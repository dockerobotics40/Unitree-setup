# Robotics 4.0 - Documentación y Recursos para Unitree

Bienvenido al repositorio de **Robotics 4.0**, un equipo dedicado a la investigación, desarrollo y aplicación de soluciones robóticas avanzadas. Nuestro objetivo es facilitar la implementación de tecnologías en robótica mediante documentación estructurada y ejemplos prácticos.

Este repositorio es una **condensación de todos los recursos disponibles** sobre la instalación y uso de repositorios de **Unitree**, además de incluir **códigos básicos creados por nuestro equipo** para facilitar el aprendizaje y la experimentación.

---

## 📁 Estructura del Repositorio

```bash
📦 unitree-setup  
.
├── README.md
├── docs
│   ├── FAQ.md
│   ├── Instalacion_driver_LiDAR_LIVOX360.md
│   ├── Instalacion_driver_depth_camera.md
│   ├── Instalacion_unitree_ros2.md
│   ├── Instalacion_unitree_sdk2.md
│   ├── Instalacion_unitree_sdk2_python.md
│   └── images
│       ├── Instalacion_unitree_ros2
│       ├── Instalacion_unitree_sdk2
│       └── Instalacion_unitree_sdk2_python
├── ejemplos
│   ├──  Secuencia_de_Ejecucion.md
│   ├── Codigo_basico_movimientoBrazo_personalizado.md
│   ├── Codigo_basico_movimientos_altoNivel.md
│   ├── Conexion_robotG1.md
│   ├── Conexion_wifi_PC2.md
│   ├── Ejemplo_acceso_sensores_ROS2.md
│   ├── Ejemplo_alto_nivel.md
│   ├── Ejemplo_bajo_nivel.md
│   ├── Ejemplo_movimiento_brazos.md
│   ├── Programas personalizados SDK python
│   │   ├── g1_arm_sdk_moveV2.py
│   │   ├── g1_moveInTime_control.py
│   │   └── g1_wasd_control.py
│   ├── Visualizacion_info_LiDAR.md
│   ├── Visualizacion_info_camaraDepth.md
│   └── images
│       ├── Conexion_robotG1
│       ├── Conexion_wifi_PC2
│       └── Ejemplo_acceso_sensores_ROS2
└── repositorios
    └── Enlaces_repositorios.md
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

📜 Licencia
Este proyecto sigue la licencia MIT.

