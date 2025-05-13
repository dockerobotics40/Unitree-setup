### **Control Combinado de Brazos y Locomoción – Robot G1 (Caso de uso llevar una caja)**

#### Objetivo General

Ejecutar una secuencia controlada donde el robot G1 se acerque a una caja, realice el agarre mediante movimientos de brazo, transporte el objeto caminando y lo deposite en un objetivo determinado. Se busca validar la integración brazo-locomoción mediante una ejecución segmentada, parametrizada y evaluable.

### 1. Preparación del Entorno

#### 1.1 Disposición Física del Espacio

* Marcar claramente sobre el suelo:
  * Punto de inicio del robot
  * Posición inicial de la caja
  * Ubicación del objetivo final
* Asegurar:
  * Caja con peso entre 0.5 kg y 1.5 kg
  * Arnés de seguridad correctamente instalado
  * Activación del modo *Main Operation Control*
  * Conexión de red activa y estable entre el G1 y la estación de control

#### 1.2 Parametrización Experimental de Trayectorias

**Objetivo:** Determinar los tiempos óptimos por segmento de locomoción, según las distancias físicas reales.

**Procedimiento:**

1. Ejecutar manualmente secuencias de caminata sin carga usando el script interactivo.
2. Medir distancias entre:
   * Punto inicial y la caja
   * Caja y el objetivo final
3. Ajustar duración de comandos de locomoción (adelante, rotar\_izq, etc.) hasta alcanzar físicamente el punto deseado.
4. Registrar los parámetros óptimos para luego diligenciar en el sistema de control.

📝 *Ejemplo de estructura en memoria para fase 1:*
`trayectoria_fase1 = [ ("adelante", 2.3), ("rotar_der", 0.8), ("adelante", 1.7) ]`

### 2. Definición de Posturas con MuJoCo

1. Cargar modelo G1 en MuJoCo:
   ```bash
   pip install mujoco
   python -m mujoco.viewer
   ```
2. Mover el robot a mano, pausar y resetear la escena para definir posturas articulares clave.
3. Registrar manualmente los valores articulares obtenidos (por ejemplo, desde la interfaz de MuJoCo) directamente en el código como diccionarios.

**Ejemplo:**

```bash
postura_fase1 = {
"RightShoulderPitch": 0.0,
"RightElbow": 1.2,
"RightWristRoll": 1.48,
...
}
```

Fases recomendadas:

* `fase1_approach`: postura de aproximación para agarre
* `fase2_grasp`: cierre de manos, contacto con la caja
* `fase3_hold`: brazos replegados con objeto sostenido
* `fase4_release`: postura para depositar la caja

### 3. Ejecución del Script `g1_armsdk_moveV5.py`

El script ejecuta cada fase de forma modular con interacción del usuario:

#### 3.1 Movimiento a Posición Cero

* El robot se posiciona en la postura neutral.
* Se solicita confirmación para continuar.

#### 3.2 Ingreso de Postura Objetivo

Opciones disponibles:

1. **Ingreso manual** de cada articulación
2. **Carga desde estructura en memoria** (por ejemplo, `postura_fase2`)
   El sistema valida nombres y valores antes de aplicar la postura.

#### 3.3 Movimiento del Brazo a Postura de Agarre

* Se aplica la postura de agarre seleccionada.
* Confirmación para continuar con la locomoción.

#### 3.4 Ejecución de Caminata (Opcional)

Opciones disponibles:

1. **Ingreso manual** de:
   * Dirección (adelante, izquierda, rotar\_der, etc.)
   * Tiempo (segundos)
2. **Carga desde estructura en memoria**, como `trayectoria_fase3`
   Cada paso es mostrado y confirmado antes de ejecutarse con `execute_trajectory_sequence()`.

#### 3.5 Repetición / Retorno / Finalización

Opciones luego de cada acción:

* Ingresar una nueva postura
* Volver a posición cero
  Luego de volver a cero:
* Ingresar otra postura
* Liberar el control del robot y salir

El script finaliza limpiamente, liberando el control del robot de forma segura.

