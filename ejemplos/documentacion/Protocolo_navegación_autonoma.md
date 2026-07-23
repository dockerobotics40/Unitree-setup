# Guía de Uso – Navegación Autónoma del Robot G1


## Objetivo General

Permitir que el robot G1 de Unitree se desplace de forma autónoma hacia múltiples puntos definidos por el usuario, utilizando únicamente su odometría y un control proporcional adaptativo. El robot corrige su orientación antes y después de cada desplazamiento y se detiene si no alcanza el objetivo dentro de un tiempo límite.

## Requisitos

* G1 encendido y en modo **Main Operation Control** (R1 + X)
* Conexión activa vía Ethernet (ej. `<span>eth0</span>`)
* SDK2 instalada correctamente (Python)
* Script `<span>g1_autonomousV1.py</span>` actualizado (sin uso de `<span>.txt</span>`)

## Ejecución

```
python3 g1_autonomousV1.py
```

Al inicio se solicita:

* **Interfaz de red** (ej. `eth0`)
* **Ingreso manual** de puntos objetivo:

```
0.5 0.0 0.0
1.0 0.5 1.57
1.0 1.0 3.14
fin
```

Cada línea representa un objetivo con:

* `x`: posición en metros
* `y`: posición en metros
* `yaw`: orientación en radianes (0 = hacia adelante)

## Funcionamiento Interno

### Para cada punto objetivo:

1. **Reorientación previa**
   * Gira hacia la dirección del siguiente objetivo
   * Si el giro requerido es muy brusco (> 45°), realiza un **retroceso adaptativo**
2. **Desplazamiento**
   * Control proporcional en `<span>x</span>`, `<span>y</span>` y `<span>yaw</span>`
   * Límites de velocidad:
     * Adelante: hasta 0.4 m/s
     * Atrás: hasta -0.15 m/s
     * Lateral: ±0.3 m/s
     * Rotación: ±0.5 rad/s
   * Timeout: 30 segundos por objetivo
3. **Reorientación final**
   * Corrige su `yaw` hasta una tolerancia de ±0.11 rad

## Notas Técnicas

* Control adaptativo tipo "P con freno": mayor precisión al acercarse
* El `yaw` se normaliza automáticamente (sin saltos entre +π y -π)
* Movimiento suave, sin necesidad de sensores externos
* Seguridad ante interrupciones: se detiene si se presiona `Ctrl+C`

## Criterios de Éxito

* El robot llega a cada punto y se orienta correctamente
* La posición final tiene un error < 20 cm (tolerancia configurable)
* El sistema responde a interrupciones sin fallar

## Sugerencia de Prueba en Campo

1. Marcar en el piso los puntos destino
2. Medir y anotar sus coordenadas relativas al punto inicial del G1
3. Ingresarlas manualmente al ejecutar el script
4. Observar su desplazamiento continuo sin intervención externa
5. Evaluar la desviación final y ajustar si es necesario

## Consejos

* Usar `matplotlib` o un sistema externo si deseas visualizar la trayectoria
* Puedes modificar el script para registrar `x, y, yaw` en cada paso
* Para trayectorias más complejas, genera estructuras en memoria y reemplaza el ingreso manual
