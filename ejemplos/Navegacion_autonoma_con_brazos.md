### Navegación Autónoma por Odometría con Múltiples Objetivos y Coordinación de Movimientos de Brazos

#### Objetivo:

Verificar que el robot G1 de Unitree pueda desplazarse de forma autónoma a una secuencia de posiciones predeterminadas utilizando exclusivamente su odometría interna, con posibilidad de ejecutar movimientos preconfigurados de los brazos en cada objetivo alcanzado.

Se utiliza el script: `g1_autonomusWithArmV11.py`

#### Requisitos Previos

* Conexión Ethernet estable entre el PC y el robot G1.
* Robot encendido en modo normal (`R1 + X`).
* Entorno Python con SDK2 de Unitree funcional.
* ROS 2 instalado y accesible (si aplica).
* Script `g1_autonomusWithArmV11.py` disponible y funcional.

#### Procedimiento

##### 1. Conexión y Configuración Inicial

1. Abre una terminal en el entorno configurado.
2. Navega hasta el directorio donde se encuentra el script.
3. Ejecuta el script con:
   ```bash
   python3 g1_autonomusWithArmV11.py
   ```
4. Ingresa la interfaz de red conectada al robot (ej. `eth0`).

##### 2. Ingreso Manual de Objetivos de Navegación

Cuando se pregunte:

```bash
¿Cargar puntos desde archivo? (s/n):
```

Responde con `n`.

Luego ingresa los objetivos uno por uno en el formato:

```bash
X Y Yaw
```

Ejemplo:

```bash
0.5 0.0 0.0
1.0 0.5 1.57
1.0 1.0 3.14
fin
```

##### 3. Coordinación de Movimientos de Brazos

En cada objetivo alcanzado, el sistema preguntará:

```bash
¿Deseas ejecutar un movimiento de brazos en este objetivo? (s/n):
```

Si respondes `s`:

1. Se mostrará:

   ```bash
   ¿Qué paso deseas ejecutar? (1-6):
   ```
2. Ingresa el número del movimiento deseado (del 1 al 6), que corresponde a una pose predefinida de brazo en `get_user_joint_positions(paso)`.
3. Luego se preguntará:

   ```bash
   ¿Deseas ejecutar otro movimiento de brazos en este objetivo? (s/n):
   ```

   * Si `s`, repite el paso anterior.
   * Si `n`, continúa con la navegación al siguiente objetivo.

##### 4. Finalización

Al completar todos los objetivos correctamente, el script imprimirá:

```bash
Todos los objetivos alcanzados y orientados correctamente.
```

Para detener la ejecución en cualquier momento, presiona `Ctrl+C`.

#### Notas Técnicas

* Se emplea control **proporcional adaptativo tipo "P con freno"**, reduciendo la ganancia en errores pequeños.
* Velocidades limitadas:
  * Lateral: ±0.2 m/s
  * Retroceso: hasta -0.15 m/s
* El valor de yaw se normaliza automáticamente para evitar discontinuidades.
* Aunque el script permite cargar objetivos desde archivo `.txt`, **en esta prueba se omite dicha funcionalidad**.
