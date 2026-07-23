# Robotics 4.0 - Documentación y Recursos para Unitree

Bienvenido al repositorio de **Robotics 4.0**, un equipo dedicado a la investigación, desarrollo y aplicación de soluciones robóticas avanzadas. Nuestro objetivo es facilitar la implementación de tecnologías en robótica mediante documentación estructurada y ejemplos prácticos.

Este repositorio es una **condensación de todos los recursos disponibles** sobre la instalación y uso de repositorios de **Unitree**, además de incluir **códigos básicos creados por nuestro equipo** para facilitar el aprendizaje y la experimentación.

---

## 📁 Estructura del Repositorio

```text
.
├── docs
│   ├── FAQ.md
│   ├── images
│   │   ├── Instalación_recursos_simulación
│   │   │   ├── 1759163230947.png
│   │   │   ├── 1759163648948.png
│   │   │   ├── 1759163678460.png
│   │   │   ├── 1759163780059.png
│   │   │   ├── 1759164148651.png
│   │   │   └── 1759164256361.png
│   │   ├── Instalacion_unitree_ros2
│   │   │   ├── 1743698231862.png
│   │   │   ├── 1743698536361.png
│   │   │   └── 1743698558563.png
│   │   ├── Instalacion_unitree_sdk2
│   │   │   ├── 1743698433382.png
│   │   │   ├── 1743698450950.png
│   │   │   ├── 1743698476310.png
│   │   │   └── 1743698495254.png
│   │   └── Instalacion_unitree_sdk2_python
│   │       └── 1743698365157.png
│   ├── Instalacion_driver_depth_camera.md
│   ├── Instalacion_driver_LiDAR_LIVOX360.md
│   ├── Instalacion_recursos_simulacion.md
│   ├── Instalacion_unitree_ros2.md
│   ├── Instalacion_unitree_sdk2.md
│   └── Instalacion_unitree_sdk2_python.md
├── ejemplos
│   ├── codigo_robot
│   │   ├── 23dof
│   │   │   ├── arm_sdk
│   │   │   │   ├── g1_23dof_physical_selector.py
│   │   │   │   └── g1_arm_sdk_moveV4.py
│   │   │   └── control_general
│   │   │       └── g1_moveInTime_control.py
│   │   └── 29dof
│   │       ├── arm_sdk
│   │       │   ├── g1_arm_sdk_moveV4.py
│   │       │   ├── g1_arm_sdk_moveV5.py
│   │       │   └── g1_arm_sdk_visualizer_pos_torque.py
│   │       └── control_general
│   │           ├── g1_autonomusV1.py
│   │           ├── g1_moveInTime_control.py
│   │           ├── g1_odometry.py
│   │           └── g1_wasd_control.py
│   ├── documentacion
│   │   ├── Codigo_basico_brazos_caminata.md
│   │   ├── Codigo_basico_trayectoria_altoNivel.md
│   │   ├── Codigo_basico_wasd_altoNivel.md
│   │   ├── Codigo_movimiento_articulaciones_brazos_interactivo.md
│   │   ├── Conexion_robotG1.md
│   │   ├── Conexion_wifi_PC2.md
│   │   ├── Ejemplo_acceso_sensores_ROS2.md
│   │   ├── Ejemplo_alto_nivel.md
│   │   ├── Ejemplo_bajo_nivel.md
│   │   ├── Ejemplo_movimiento_brazos.md
│   │   ├── Protocolo_navegación_autonoma.md
│   │   ├── Secuencia_de_Ejecucion.md
│   │   └── Simulacion_G1_Mujoco.md
│   ├── images
│   │   ├── Conexion_robotG1
│   │   │   ├── 1743698697251.png
│   │   │   ├── 1743698725476.png
│   │   │   └── 1743698739924.png
│   │   ├── Conexion_wifi_PC2
│   │   │   └── 1743698806090.png
│   │   ├── Ejemplo_acceso_sensores_ROS2
│   │   │   ├── 1743698878139.png
│   │   │   ├── 1743698953466.png
│   │   │   └── 1743698986524.png
│   │   └── Simulacion_G1_Mujoco
│   │       ├── 1759167980800.png
│   │       ├── 1759168040102.png
│   │       ├── 1759168104008.png
│   │       └── 1759168133309.png
│   └── simulacion_mujoco
│       ├── 23dof
│       │   ├── poses
│       │   │   ├── 0_pose_segura.json
│       │   │   ├── 1_saludo_derecha.json
│       │   │   ├── 2_saludo_formal.json
│       │   │   ├── 3_saludo_izq.json
│       │   │   ├── 4_boxeo.json
│       │   │   └── 5_dab.json
│       │   └── scripts
│       │       └── extras
│       │           ├── capture_pose_mujoco_23dof.py
│       │           ├── g1_23dof_mujoco_selector.py
│       │           └── play_pose_mujoco_23dof.py
│       └── 29dof
│           ├── poses
│           │   ├── aplaudir.txt
│           │   └── saludoR.txt
│           └── scripts
│               ├── g1_arms_example.py
│               ├── g1_low_level_example.py
│               └── test_unitree_sdk2_mod.py
├── LICENSE
├── README.md
└── repositorios
    └── Enlaces_repositorios.md

29 directories, 72 files
```

## 📌 Contenidos

### 1. Instalaciones

Aquí encontrarás guías detalladas para instalar:

- SDK2 de Unitree para python y C++
- Unitree_ROS2 para manejo de G1 desde ROS2
- Drivers para uso del LiDAR y cámara integrados
- Entorno de simulación en Mujoco para validar desarrollos y controladores personalizados.

📍 Para empezar, consulta la guía de instalación del SDK2 de Unitree en [docs/Instalacion_unitree_SDK2.md](docs/Instalacion_unitree_sdk2.md).

### 2. Ejemplos Básicos y personalizables

Una vez instalado el SDK2 en python/C++, paquete de ROS2 para Unitree, drivers y configurado el entorno, puedes probar ejemplos como:

- Control de bajo nivel del robot (movimiento de sus tobillos).
- Control de alto nivel usando la API de unitree.
- Control de movimientos personaliados de los brazos.
- Visualización de sensores exteroceptivos.
- Conexión WiFi desde el computador de desarrollo del robot.

📚 Puedes encontrar los ejemplos en la carpeta examples/.

### 🤝 [Conocer al Equipo](https://robotics40.com/)

Somos Robotics 4.0, una empresa lider en robótica en Colombia que busca innovar y desarrollar herramientas accesibles para la comunidad.

### 📜 Licencia

El código de este repositorio se distribuye bajo la **Licencia Apache 2.0**.
Esto significa que puedes usarlo, modificarlo y redistribuirlo libremente, siempre y cuando mantengas el aviso de copyright y la referencia a Robotics 4.0.

**Nota importante:**
Este repositorio contiene **ejemplos y guías de uso**. No representa el producto completo desarrollado por Robotics 4.0.
Para soluciones empresariales completas, soporte o integración, por favor contáctanos en: contacto@robotics40.com.

