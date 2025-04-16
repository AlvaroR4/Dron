# Scripts de Prueba de Distancia (Simulación)

Dos scripts para probar y comparar la medición de distancia a un objeto (puerta).
Modificado para usar ANCHO de la puerta en vez de la altura (mas preciso por los postes de las puertas) 
## Archivos

1.  **`prueba_distancia_mover.py`**:
    * **Función:** Script MAVSDK. Conecta al dron, despega, mide continuamente la distancia real (ground truth) a una puerta objetivo y la imprime en consola.
    * **Configuración Requerida:**
        * `GATE_POS_NED`: Coordenadas [Norte, Este, Abajo] (metros) del centro de la puerta objetivo.

2.  **`pruebas_distancia_tratar.py`**:
    * **Función:** Nodo ROS 2. Procesa imagen de cámara simulada, detecta objeto por color, estima la distancia basada en tamaño aparente y la imprime en consola/logs.
    * **Configuración Requerida:**
        * `TAMANO_REAL_PUERTA_M`
        * `DISTANCIA_FOCAL_PIXELS`: Distancia focal (`fx` o `fy`, en píxeles) de la cámara simulada.

## Uso Básico

1.  Rellenar las variables/constantes globales en ambos archivos con los valores correctos.
2.  Lanzar simulación Gazebo+PX4 con el mundo deseado.
3.  Ejecutar `pruebas_distancia_tratar.py` (nodo ROS 2).
4.  Ejecutar `prueba_distancia_mover.py` (script Python).
5.  Comparar las distancias impresas.