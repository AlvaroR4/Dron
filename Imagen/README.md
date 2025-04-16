# Imagen
# Control de drones mediante la cámara


## Códigos que funcionan:

```
Imagen/
├── tratarImagen2.py
├── tratarImagen5.py
├── tratarImagen6.py
├── mover2.py
├── mover3.py
├── camaraReal/
|   ├── publicador_camara.py
|   ├── tratarImagen3.py
|   ├── tratarImagen4.py
├── pruebasDistancia/
|
├── versionesAntiguas/
|   ├── mover.py
|   ├── tratarImagen2.py
|   ├── tratarImagen5.py

```
### 🏎️ tratarImagen2.py
Este script recibe el topic ROS2 de una cámara y la procesa
Busca objetos de color rojo y calcula el offset a su centroide
Una vez calculado el centroide, envia offset_x y offset_y a través de un socket
Utiliza hilos para enviar las coordenadas cada x segs

### 🚁 tratarImagen3.py {Cámara real}
Este script recibe el topic de una cámara real y la procesa
Busca objetos de color rojo y calcula el offset a su centroide
Una vez calculado el centroide, envia offset_x y offset_y a través de un socket
Utiliza hilos para enviar las coordenadas cada x segs

### 🏎️ tratarImagen4.py {Cámara real}
Este script recibe el topic ROS2 de una cámara y la procesa
Busca objetos de color rojo y calcula el offset a su centroide
Una vez calculado el centroide, envia offset_x y offset_y a través de un socket

### 🏎️ tratarImagen5.py
Este script recibe el topic ROS2 de una cámara y la procesa
Busca objetos de color rojo y calcula el offset a su centroide
Una vez calculado el centroide, envia offset_x y offset_y a través de un socket

### 🏎️ tratarImagen5.py
Este script recibe el topic ROS2 de una cámara y la procesa
Busca objetos de color rojo y calcula el offset a su centroide y el área del objeto
Una vez calculado el centroide, envia offset_x , offset_y y área a través de un socket

### 🚁 mover.py
Este script recibe coordenadas x,y a través de un socket
Trata de alinear el dron respecto al centroide de un objeto para pasar a través de él
Usa estados para alinear primero un eje

### 🚁 mover2.py (En desarrollo)
Este script recibe coordenadas x,y a través de un socket
Trata de alinear el dron respecto al centroide de un objeto para pasar a través de él
Trata de alinear los dos ejes a la vez

### 🚁 mover3.py (En desarrollo)
Este script recibe coordenadas x,y y el área a través de un socket
Trata de alinear el dron respecto al centroide de un objeto para pasar a través de él
Usa estados para alinear primero un eje
Sigue alineandose cuando avanza hacia la puerta

### 🏎️ publicador_camara.py
Este script publica en un topic las imagenes de una cámara física
Se puede elegir la propia cámara del pc, o una externa

### /pruebasDistancia
Carpeta con scrpts para realizar pruebas del cálculo de distancias con cámaras.


## Ejemplos de simulación
Comandos de ejecución en:

```
Dron/
│── Guias/
│   ├── comandos.txt
```

Lectura simple del topic de la cámara: 

---PARA VISUALIZAR LA CÁMARA CON ROS2 Y RQT
Lanzar dron con cámara;  
make px4_sitl gz_x500_mono_cam


Ejecutar el agente para que se conecte con el cliente del dron; 
MicroXRCEAgent udp4 -p 8888


---Relacionar las carpetas de intalación(ws->puentes ros2; ws_sensor->controladora dron)
source ~/ws/install/setup.sh
source ~/ws_sensor_combined/install/setup.sh


Lanzar el brigde de ros2 con la cámara; 
ros2 run ros_gz_image image_bridge /world/default/model/x500_mono_cam_0/link/camera_link/sensor/imager/image


Abrir rqt y visualizar el topic de la cámara; (Para visualizar el gazebo suscribirse a image display)



## Autor
Álvaro Ros - Simulación con PX4, Gazebo , MAVSDK y ROS2

## Licencia
Este proyecto está bajo la **Licencia MIT**. Consulta el archivo `LICENSE` para más detalles.
