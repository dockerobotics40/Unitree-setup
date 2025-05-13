## Protocolo de Navegación Autónoma del G1

### Prueba 1: Control Semiteledirigido del Brazo

**Objetivo:**
Controlar manualmente las articulaciones del brazo del G1 usando `arm_sdk`, permitiendo definir poses personalizadas para manipulación.

**Requisitos previos:**

* Acceso al script `RUTINA_BRAZOS.py`.

**Procedimiento:**

1. Ejecutar el script:
   ```bash
   python3 g1_arm_sdk_moveV4.py <interfaz_red>
   ```
2. Mover el brazo a distintas poses manualmente según las pautas asignadas, se modifican en el código las posiciones deseadas.

### Prueba 2: Registro de Odometría y Visualización de Trayectoria

**Objetivo:**
Evaluar la precisión de la odometría del G1 durante un trayecto físico, observando en consola su posición, orientación y velocidad.

**Requisitos previos:**

* Conexión Ethernet activa con el G1.
* SDK2 de Unitree en Python instalada.
* Robot en modo normal y operación principal.
* Script funcional `g1_odometry.py`.

**Procedimiento:**

1. Preparar un entorno libre de obstáculos con marcas visibles en el suelo.
2. Ejecutar el script:
   
   ```bash
   python3 g1_odometry.py <interfaz_red>
   ```
3. Con el control remoto, desplazar el robot por la ruta marcada.
4. Observar en consola cada 500 ciclos:
   
   * Posición: `x`, `y`, `z`
   * Orientación: `roll`, `pitch`, `yaw`
   * Velocidad lineal y angular
5. Registrar manualmente las coordenadas clave (inicio, giros, destino).

**Visualización opcional (posterior):**
Modificar el script para guardar `x_values`, `y_values` y graficar:

```bash
import matplotlib.pyplot as plt
plt.plot(x_values, y_values)
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Trayectoria registrada por odometría")
plt.grid(True)
plt.show()
```

**Criterios de éxito:**

* Los datos reflejan correctamente los movimientos del robot.
* La trayectoria es continua y lógica.
* Se identifican puntos relevantes para navegación futura.

**Solución de problemas:**

| Problema              | Causa probable                        | Acción recomendada                           |
| --------------------- | ------------------------------------- | --------------------------------------------- |
| Odometría en 0       | SportModeState\_ sin actualizar       | Verificar modo del robot y conexión Ethernet |
| Datos congelados      | Falta de activación de`first_update` | Confirmar suscripción a`rt/lowstate`         |
| Desfase en yaw        | Giro manual sin referencia            | Usar marcas físicas para referencia          |
| No imprime odometría | `counter_`muy alto                    | Reducir la frecuencia de impresión           |

### Prueba 3: Navegación Autónoma con Múltiples Objetivos

**Objetivo:**
Comprobar que el G1 puede desplazarse autónomamente entre múltiples puntos usando únicamente su odometría y el script `g1_autonomousV1.py`.

**Procedimiento:**

1. Establecer conexión:
   
   * Conectar por Ethernet.
   * Encender el robot (R1 + X).
   * Verificar entorno Python con SDK2.
2. Ejecutar el script:
   
   ```bash
   python3 g1_autonomousV1.py
   ```
   
   * Ingresar la interfaz de red (`eth0`, `enp0s31f6`, etc).
   * Indicar **"no"** cuando se pregunte por carga desde archivo.
   * Ingresar manualmente los puntos objetivo:
     ```bash
     0.5 0.0 0.0
     1.0 0.5 1.57
     1.0 1.0 3.14
     fin
     ```
   
3. El robot debe:
   
   * Orientarse hacia el objetivo.
   * Avanzar con control proporcional adaptativo.
   * Realinearse al llegar.
   * Repetir para cada punto.
4. Finalizar:
   
   * El robot debe detenerse automáticamente al concluir la secuencia.
   * Evaluar precisión del arribo a cada punto.

**Notas técnicas:**

* Control tipo "P con freno" (reduce ganancia al acercarse).
* Movimiento lateral limitado a ±0.2 m/s; retroceso máximo -0.15 m/s.
* Yaw normalizado automáticamente para evitar saltos.

