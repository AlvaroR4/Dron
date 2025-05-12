nodo_camara_tello.py:

Se conecta al Tello y obtiene el stream de vídeo.
Publica la imagen RAW (BGR) del Tello en el topic /tello/imagen_raw.
Se suscribe al topic /tello/comandos_velocidad (tipo std_msgs/Float32MultiArray) para recibir [vel_lateral, vel_frontal, vel_vertical, vel_yaw].
Envía estos comandos al Tello usando tello.send_rc_control().
Manejo de Teclado:
f: Llama a tello.emergency() (parada inmediata de motores).
Ctrl+C: Llama a tello.land() y cierra el nodo.
Mínimos prints, solo para errores críticos o estado de conexión inicial.
Despega al inicio.

nodo_procesador_imagen.py:

Se suscribe a /tello/imagen_raw.
Aplica toda la lógica de tratarTello_ANTIGUO.py para detectar la puerta roja (conversión HSV, máscaras, contornos, cálculo de offset en píxeles, cálculo de distancia).
Publica la imagen procesada con las detecciones (rectángulos, texto) en /tello/imagen_procesada (tipo sensor_msgs/Image, codificación RGB8).
Publica los datos de detección: offset_x_normalizado, offset_y_normalizado, distancia_metros, numero_objetivos_validos en el topic /tello/datos_deteccion 

nodo_control_logica.py:

Se suscribe a /tello/datos_deteccion.
Implementa toda la lógica de estados y movimiento de moverTello_ANTIGUO.py.
Basándose en los datos recibidos y el estado actual, calcula los cuatro valores de velocidad: [vel_lateral, vel_frontal, vel_vertical, vel_yaw].
Publica estos cuatro valores en /tello/comandos_velocidad.
Este nodo contendrá los prints para seguir la lógica, estados, etc.
