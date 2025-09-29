# Robotics 4.0 - Documentación y Recursos para Unitree

Bienvenido al repositorio de **Robotics 4.0**, un equipo dedicado a la investigación, desarrollo y aplicación de soluciones robóticas avanzadas. Nuestro objetivo es facilitar la implementación de tecnologías en robótica mediante documentación estructurada y ejemplos prácticos.

Este repositorio es una **condensación de todos los recursos disponibles** sobre la instalación y uso de repositorios de **Unitree**, además de incluir **códigos básicos creados por nuestro equipo** para facilitar el aprendizaje y la experimentación.

---

## 📁 Estructura del Repositorio

```bash
📦 unitree-setup  
.
├── LICENSE
├── README.md
├── docs
│   ├── FAQ.md
│   ├── Instalacion_driver_LiDAR_LIVOX360.md
│   ├── Instalacion_driver_depth_camera.md
│   ├── Instalacion_recursos_simulacion.md
│   ├── Instalacion_unitree_ros2.md
│   ├── Instalacion_unitree_sdk2.md
│   ├── Instalacion_unitree_sdk2_python.md
├── ejemplos
│   ├──  Secuencia_de_Ejecucion.md
│   ├── Codigo_basico_brazos_caminata.md
│   ├── Codigo_basico_trayectoria_altoNivel copy.md
│   ├── Codigo_basico_wasd_altoNivel.md
│   ├── Codigo_movimento_articulaciones_brazos_interactivo.md
│   ├── Codigos personalizados
│   │   ├── g1_arm_sdk_moveV4.py
│   │   ├── g1_arm_sdk_moveV5.py
│   │   ├── g1_arm_sdk_visualizer_pos_torque.py
│   │   ├── g1_autonomusV1.py
│   │   ├── g1_moveInTime_control.py
│   │   ├── g1_odometry.py
│   │   └── g1_wasd_control.py
│   ├── Codigos simulacion
│   │   ├── aplaudir.txt
│   │   ├── g1_arms_example.py
│   │   ├── g1_low_level_example.py
│   │   ├── saludoR.txt
│   │   └── test_unitree_sdk2_mod.py
│   ├── Conexion_robotG1.md
│   ├── Conexion_wifi_PC2.md
│   ├── Ejemplo_acceso_sensores_ROS2.md
│   ├── Ejemplo_alto_nivel.md
│   ├── Ejemplo_bajo_nivel.md
│   ├── Ejemplo_movimiento_brazos.md
│   ├── Protocolo_navegación_autonoma.md
│   ├── Simulacion_G1_Mujoco.md
│   └── images
│       ├── Conexion_robotG1
│       │   ├── 1743698697251.png
│       │   ├── 1743698725476.png
│       │   └── 1743698739924.png
│       ├── Conexion_wifi_PC2
│       │   └── 1743698806090.png
│       ├── Ejemplo_acceso_sensores_ROS2
│       │   ├── 1743698878139.png
│       │   ├── 1743698953466.png
│       │   └── 1743698986524.png
│       └── Simulacion_G1_Mujoco
│           ├── 1759167980800.png
│           ├── 1759168040102.png
│           ├── 1759168104008.png
│           └── 1759168133309.png
└── repositorios
    └── Enlaces_repositorios.md
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

