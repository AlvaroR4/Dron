# Procesamiento de Cámara Real con ROS 2 y OpenCV

Este proyecto demuestra cómo capturar vídeo desde una cámara de PC (integrada o USB) usando Python y OpenCV, publicarlo como un topic de imagen en ROS 2, y luego suscribirse a ese topic con otro nodo para procesar la imagen (detectar un color específico, calcular su centroide y el offset respecto al centro de la imagen) y enviar este offset vía UDP.

Utiliza dos scripts principales:
* `publicador_camara.py`: Lee la cámara y publica las imágenes en ROS 2.
* `tratarImagen3.py`: Recibe las imágenes de ROS 2, las procesa y envía datos por UDP.

## Requisitos

* **Dependencias Python:** `opencv-python`
    ```bash
    pip install opencv-python
    # O si prefieres la versión de los repositorios de Ubuntu (puede ser más antigua):
    # sudo apt update
    # sudo apt install python3-opencv
    ```
* **Dependencias ROS 2:** `cv_bridge`
    ```bash
    sudo apt update
    sudo apt install ros-$ROS_DISTRO-cv-bridge
    ```
* **Utilidades de Sistema:** `v4l-utils` (para identificar la cámara)
    ```bash
    sudo apt update
    sudo apt install v4l-utils
    ```

## Configuración y Uso

Sigue estos pasos para ejecutar el sistema:

**1. Activar Entorno ROS 2:**
   Asegúrate de activar tu entorno ROS 2 en **CADA NUEVA TERMINAL** que abras para ejecutar los nodos.
   ```bash
   source /opt/ros/$ROS_DISTRO/setup.bash
   # Si usas un workspace compilado, actívalo también:
   # source install/setup.bash
   ```

**2. Identificar el Índice de tu Cámara:**
   Necesitamos saber qué índice numérico usa el sistema para tu cámara (normalmente `0` para la integrada).
   * Abre una terminal.
   * Ejecuta:
        ```bash
        v4l2-ctl --list-devices
        ```
   * Busca tu cámara en la lista (ej. "Integrated Camera") y anota el número `X` del dispositivo asociado `/dev/videoX`. Este `X` es tu `indice_camara`.

**3. Ejecutar el Nodo Publicador (`publicador_camara.py`):**
   Este nodo leerá la cámara y publicará las imágenes.
   * Ejecuta el script:
        * Si tu cámara usa el índice `0` (lo más común):
            ```bash
            python3 publicador_camara.py
            ```
        * Si tu cámara usa otro índice (ej. `1`):
            ```bash
            python3 publicador_camara.py --ros-args -p indice_camara:=1
            ```
   * **Importante:** Fíjate en la salida del script, indicará en qué topic está publicando. Por defecto es `/camara_real/image_raw`. Puedes cambiarlo con:
        ```bash
        # Ejemplo para publicar en otro topic y con índice 1
        python3 publicador_camara.py --ros-args -p indice_camara:=1 -p topic_imagen:=/mi_cam/imagen
        ```
   * Deja esta terminal ejecutando el script.

**4. Ejecutar el Nodo Procesador (`tratarImagen3.py`):**
   Este nodo recibirá las imágenes publicadas, las procesará y enviará datos UDP.
   * Ejecuta el script:
        * Si el publicador está usando el topic por defecto (`/camara_real/image_raw`):
            ```bash
            python3 tratarImagen3.py
            ```
        * Si el publicador está usando un topic diferente (ej. `/mi_cam/imagen`):
            ```bash
            python3 tratarImagen3.py --ros-args -p topic_entrada_imagen:=/mi_cam/imagen
            ```
   * **Asegúrate** de que el valor de `topic_entrada_imagen` aquí **coincide exactamente** con el topic que está usando `publicador_camara.py`.

**5. Observar:**
   * Debería aparecer una ventana de OpenCV titulada "Imagen Procesada (Real)" mostrando el vídeo de tu cámara con posibles detecciones.
   * El nodo procesador estará enviando mensajes UDP con las coordenadas del offset a `127.0.0.1:65432` (puedes cambiar esto editando las constantes `SERVER_IP` y `SERVER_PORT` en `tratarImagen3.py`).

## Resumen de los Scripts

* **`publicador_camara.py`:**
    * Se conecta a la cámara física especificada por `indice_camara`.
    * Captura frames a la `frecuencia` definida.
    * Convierte los frames a mensajes `sensor_msgs/msg/Image` usando `cv_bridge`.
    * Publica los mensajes en el `topic_imagen` especificado.
* **`tratarImagen3.py`:**
    * Se suscribe al `topic_entrada_imagen`.
    * Cuando recibe un mensaje de imagen, lo convierte a formato OpenCV (BGR).
    * Aplica una máscara de color (definida por `lower_color`, `upper_color` - **¡recuerda ajustarlas a tu necesidad y que están en BGR!**) para detectar objetos.
    * Calcula el centroide del objeto más grande detectado (si lo hay) y su offset respecto al centro de la imagen.
    * Muestra la imagen con las detecciones en una ventana OpenCV.
    * Envía el `offset_x` y `offset_y` calculados a través de un socket UDP a una dirección y puerto fijos.

## Configuración Adicional

* **Detección de Color:** Edita las variables `lower_color` y `upper_color` dentro de `tratarImagen3.py` para ajustar el rango del color (en formato BGR) que quieres detectar.
* **Destino UDP:** Edita las constantes `SERVER_IP` y `SERVER_PORT` en `tratarImagen3.py` si necesitas enviar los datos a otra dirección o puerto.