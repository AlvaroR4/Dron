# Nodo ROS 2 que se conecta al Tello, detecta puertas rojas (usando HSV),
# calcula offset/distancia, envía datos por UDP
# y PUBLICA la imagen (convertida a RGB) con las detecciones en un topic ROS 2.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from djitellopy import Tello
import cv2
import numpy as np
import socket
import time
import math # No se usa directamente, pero calcular_distancia podría necesitarlo
import traceback
# import threading # No se usa explícitamente en el código proporcionado

# --- Constantes ---
# Comunicación UDP (a moverDron_tello.py o similar)
SERVER_IP = "127.0.0.1"
SERVER_PORT = 65432
# Topics ROS
ROS_TOPIC_DISPLAY_OUTPUT = '/tello/image_processed_rgb' # Topic para la imagen RGB con detecciones
# Tello
TELLO_IP = '192.168.10.1' # IP por defecto del Tello en modo Access Point
# Bucle y Procesamiento
TIMER_PERIOD = 1.0 / 20.0 # Procesar a ~20 FPS (ajustar según capacidad)
FRAME_WIDTH_PROC = 640    # Ancho de la imagen para procesamiento
FRAME_HEIGHT_PROC = 480   # Alto de la imagen para procesamiento
FRAMES_TIMEOUT_NONE = 600 # Número de frames None consecutivos antes de abortar

# --- Constantes de Visión (¡¡¡ CALIBRAR / AJUSTAR ESTAS !!!) ---
TAMANO_REAL_PUERTA_M = 1.5         # Tamaño real del objeto (puerta/marco) en metros (altura o ancho)
DISTANCIA_FOCAL_PIXELS_TELLO = 920 # Distancia focal de la cámara del Tello en píxeles. ¡CALIBRAR!
                                   # Este valor depende de la resolución usada para la calibración.
                                   # Si FRAME_WIDTH_PROC/FRAME_HEIGHT_PROC es diferente a la calibración, ajustar.
MIN_CONTOUR_AREA_TELLO = 2000      # Área mínima del contorno para ser considerado una detección válida. ¡AJUSTAR!

# Rangos HSV para ROJO (¡¡AJUSTAR CON CALIBRADOR HSV!!)
# El rojo a menudo cruza el límite 0/179 en el espacio HUE de OpenCV
COLOR_LOWER_1 = np.array([0, 130, 90])   # Límite inferior para el primer rango de rojo
COLOR_UPPER_1 = np.array([10, 255, 255])  # Límite superior para el primer rango de rojo
COLOR_LOWER_2 = np.array([160, 130, 90]) # Límite inferior para el segundo rango de rojo
COLOR_UPPER_2 = np.array([179, 255, 255]) # Límite superior para el segundo rango de rojo
# --- Fin Constantes ---

def calcular_distancia(tamanio_aparente_pixels, distancia_focal_pixels, tamano_real_objeto_m):
    """
    Calcula la distancia a un objeto basándose en su tamaño aparente en la imagen.
    Asegúrate de que tamano_real_objeto_m y el eje usado para tamanio_aparente_pixels
    (ej. alto o ancho) sean consistentes.
    """
    if tamanio_aparente_pixels <= 1: # Evitar división por cero o valores muy pequeños
        return float('inf')
    return (tamano_real_objeto_m * distancia_focal_pixels) / tamanio_aparente_pixels

class TelloRedDetectorRGBPublisher(Node): # Nombre de clase actualizado
    def __init__(self):
        super().__init__('tello_red_detector_rgb_publisher') # Nombre del nodo actualizado
        self.get_logger().info("Iniciando Nodo Detector de Marcos Rojos y Publicador RGB del Tello.")
        self.get_logger().info(f"Publicando visualización RGB en: {ROS_TOPIC_DISPLAY_OUTPUT}")
        self.get_logger().info(f"Enviando datos de control a UDP {SERVER_IP}:{SERVER_PORT}")

        self.bridge = CvBridge()

        # Configurar Socket UDP
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_address = (SERVER_IP, SERVER_PORT)

        # Configurar Tello
        self.tello = Tello(host=TELLO_IP)
        self.frame_reader = None
        self.tello_connected = False
        self.stream_on = False
        try:
            self.get_logger().info(f"Conectando al Tello en {TELLO_IP}...")
            self.tello.connect()
            self.tello_connected = True
            self.get_logger().info(f"Conectado. Batería: {self.tello.get_battery()}%")

            self.get_logger().info("Iniciando stream de vídeo...")
            self.tello.streamoff() # Apagar por si estaba encendido
            time.sleep(0.5)        # Pequeña pausa
            self.tello.streamon()
            self.stream_on = True
            self.frame_reader = self.tello.get_frame_read()
            if self.frame_reader is None:
                raise RuntimeError("frame_reader es None después de streamon().")
            self.get_logger().info("Stream activado y frame_reader obtenido.")
            time.sleep(1.5) # Pausa para estabilizar el stream
        except Exception as e:
            self.get_logger().fatal(f"Error inicializando Tello: {e}")
            traceback.print_exc()
            # Intentar limpieza antes de salir
            if hasattr(self, 'tello') and self.tello_connected:
                 try:
                     if self.stream_on: self.tello.streamoff()
                     self.tello.end()
                 except Exception as e_cleanup:
                     self.get_logger().error(f"Error durante limpieza de Tello en __init__: {e_cleanup}")
            rclpy.shutdown()
            raise SystemExit("Fallo en inicialización de Tello.")

        # Crear Publicador ROS para la imagen procesada (RGB)
        qos_profile_display = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # BEST_EFFORT es común para vídeo
            history=HistoryPolicy.KEEP_LAST,
            depth=5  # Mantener pocos mensajes si el subscriber es lento
        )
        self.display_publisher_ = self.create_publisher(Image, ROS_TOPIC_DISPLAY_OUTPUT, qos_profile_display)
        self.get_logger().info("Publicador ROS (RGB) creado.")

        # Crear Temporizador ROS para el bucle de procesamiento
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)
        self.get_logger().info(f"Temporizador creado con periodo: {TIMER_PERIOD:.3f}s (~{(1.0/TIMER_PERIOD):.1f} FPS)")
        
        # Contadores para frames
        self.none_frame_streak = 0
        self.frame_counter_validos = 0


    def timer_callback(self):
        if self.frame_reader is None or not self.tello_connected:
            self.get_logger().warn("frame_reader no disponible o Tello no conectado en timer_callback.", throttle_duration_sec=5.0)
            return
        
        if self.frame_reader.stopped:
            self.get_logger().error("El lector de frames del Tello se ha detenido. Cerrando nodo.")
            self.cleanup_tello() # Llama a la función de limpieza
            rclpy.shutdown()
            return

        current_frame_bgr_raw = self.frame_reader.frame # El frame del Tello es BGR

        if current_frame_bgr_raw is None:
            self.none_frame_streak += 1
            # Loguear con menos frecuencia para no inundar la consola
            if self.none_frame_streak == 1 or self.none_frame_streak % 60 == 0:
                 is_thread_alive_str = 'N/A'
                 if hasattr(self.frame_reader, 'thread') and self.frame_reader.thread is not None:
                     is_thread_alive_str = str(self.frame_reader.thread.is_alive())
                 self.get_logger().warn(f"Frame es None (Streak: {self.none_frame_streak}). Lector vivo: {is_thread_alive_str}", throttle_duration_sec=5.0)
            
            if self.none_frame_streak > FRAMES_TIMEOUT_NONE:
                 self.get_logger().error(f"Timeout por {FRAMES_TIMEOUT_NONE} frames None consecutivos. Deteniendo nodo.")
                 self.cleanup_tello() # Llama a la función de limpieza
                 rclpy.shutdown()
                 return
            return # Salir del callback si no hay frame
        
        # Si llegamos aquí, el frame es válido
        self.none_frame_streak = 0
        self.frame_counter_validos += 1

        try:
            # --- Procesamiento de Imagen ---
            # Redimensionar el frame BGR original
            img_proc_bgr = cv2.resize(current_frame_bgr_raw, (FRAME_WIDTH_PROC, FRAME_HEIGHT_PROC))
            # Crear una copia para dibujar sobre ella (aún en BGR)
            img_proc = cv2.cvtColor(img_proc_bgr, cv2.COLOR_BGR2RGB)

            # 1. Segmentación de Color Rojo usando HSV
            # Convertir la imagen de trabajo BGR a HSV
            img_hsv = cv2.cvtColor(img_proc, cv2.COLOR_BGR2HSV)
            
            # Aplicar máscaras para el color rojo
            mask1_red = cv2.inRange(img_hsv, COLOR_LOWER_1, COLOR_UPPER_1)
            mask2_red = cv2.inRange(img_hsv, COLOR_LOWER_2, COLOR_UPPER_2)
            final_mask_red = cv2.bitwise_or(mask1_red, mask2_red)
            
            # Opcional: Aplicar operaciones morfológicas para limpiar la máscara
            # kernel_morph = np.ones((3,3),np.uint8) # Kernel más pequeño
            # final_mask_red = cv2.morphologyEx(final_mask_red, cv2.MORPH_OPEN, kernel_morph, iterations=1)
            # final_mask_red = cv2.morphologyEx(final_mask_red, cv2.MORPH_CLOSE, kernel_morph, iterations=2)

            # 2. Encontrar Contornos en la máscara final
            contours, _ = cv2.findContours(final_mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # 3. Análisis de Contornos para encontrar el "marco" o "puerta"
            detected_targets_info = []
            frame_center_x = FRAME_WIDTH_PROC // 2
            # frame_center_y = FRAME_HEIGHT_PROC // 2 # No se usa offset_y en UDP, pero útil para logs

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > MIN_CONTOUR_AREA_TELLO:
                    # Calcular momentos para obtener el centroide
                    M = cv2.moments(cnt)
                    if M['m00'] > 0: # Evitar división por cero
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        offset_x_pixels = cx - frame_center_x
                        offset_y_pixels = cy - (FRAME_HEIGHT_PROC // 2) # Centro Y del frame

                        # Usar minAreaRect para obtener un rectángulo que encierre el contorno
                        # Esto puede ser más robusto para la estimación de tamaño/distancia que boundingRect
                        rect_min_area = cv2.minAreaRect(cnt)
                        # El tamaño del rectángulo es rect_min_area[1] = (ancho, alto)
                        # Usamos el mayor de los dos (o el que corresponda a TAMANO_REAL_PUERTA_M)
                        # Para una puerta, usualmente el alto es más consistente o el ancho si es cuadrado
                        (box_w, box_h) = rect_min_area[1]
                        # Decidir si usar ancho o alto para 'tamanio_aparente_pixels'
                        # Si TAMANO_REAL_PUERTA_M es la altura, usar box_h. Si es ancho, box_w.
                        # Asumamos que TAMANO_REAL_PUERTA_M se refiere a la altura para este ejemplo.
                        tamanio_aparente_pixels_para_dist = max(box_w, box_h) # O min(), o uno específico. ¡Ajustar!
                        
                        distancia_m = calcular_distancia(tamanio_aparente_pixels_para_dist,
                                                         DISTANCIA_FOCAL_PIXELS_TELLO,
                                                         TAMANO_REAL_PUERTA_M)
                        
                        detected_targets_info.append({
                            'cx': cx, 'cy': cy, 
                            'offset_x': offset_x_pixels, 'offset_y': offset_y_pixels, # Guardamos ambos offsets
                            'distance': distancia_m, 
                            'area': area, 
                            'rect_obj': rect_min_area # Guardamos el objeto rectángulo para dibujar
                        })

            # 4. Selección del Mejor Objetivo (el más cercano) y Preparación de Datos UDP
            udp_offset_x_norm, udp_offset_y_norm, udp_distancia_m = 0.0, 0.0, float('inf')
            num_valid_targets = len(detected_targets_info)

            if num_valid_targets > 0:
                # Ordenar los objetivos detectados por distancia (el más cercano primero)
                detected_targets_info.sort(key=lambda t: t['distance'])
                closest_target = detected_targets_info[0]
                
                # Normalizar offset_x: (-1 a 1), donde -1 es extremo izquierdo, 1 extremo derecho
                udp_offset_x_norm = closest_target['offset_x'] / (FRAME_WIDTH_PROC / 2.0)
                # Normalizar offset_y: (-1 a 1), donde -1 es extremo superior, 1 extremo inferior
                udp_offset_y_norm = closest_target['offset_y'] / (FRAME_HEIGHT_PROC / 2.0)
                udp_distancia_m = closest_target['distance']

                # Dibujar el rectángulo y el centroide del objetivo más cercano en la imagen BGR
                points = cv2.boxPoints(closest_target['rect_obj'])
                points = np.intp(points) # Convertir a enteros para dibujar
                cv2.drawContours(img_proc, [points], -1, (0, 255, 0), 2) # Verde para el marco
                cv2.circle(img_proc, (closest_target['cx'], closest_target['cy']), 5, (0, 0, 255), -1) # Rojo para el centro
                
                info_text = f"D:{closest_target['distance']:.1f}m Ox:{udp_offset_x_norm:.2f} Oy:{udp_offset_y_norm:.2f}"
                cv2.putText(img_proc, info_text,
                            (closest_target['cx'] - 60, closest_target['cy'] - 20), # Posición del texto
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA) # Amarillo

            # 5. Enviar Datos por UDP
            # Formato: offset_x_normalizado,offset_y_normalizado,distancia_metros,numero_objetivos_validos
            message_udp = f"{udp_offset_x_norm:.3f},{udp_offset_y_norm:.3f},{udp_distancia_m:.2f},{num_valid_targets}"
            try:
                self.udp_socket.sendto(message_udp.encode('utf-8'), self.server_address)
            except Exception as e_udp:
                 self.get_logger().error(f"Error enviando UDP: {e_udp}", throttle_duration_sec=5.0)

            # 6. Preparar y Publicar Imagen Procesada en ROS (como RGB8)
            try:
                # Convertir la imagen BGR (con dibujos) a RGB para publicación
                img_publish_rgb = cv2.cvtColor(img_proc, cv2.COLOR_BGR2RGB)
                
                # Crear mensaje ROS desde la imagen RGB
                ros_image_msg_out = self.bridge.cv2_to_imgmsg(img_publish_rgb, encoding="rgb8")
                ros_image_msg_out.header.stamp = self.get_clock().now().to_msg()
                ros_image_msg_out.header.frame_id = "tello_camera_processed_rgb" # Identificador del frame
                self.display_publisher_.publish(ros_image_msg_out)
            except CvBridgeError as e_cv_bridge:
                 self.get_logger().error(f"Error CvBridge al convertir/publicar imagen: {e_cv_bridge}")
            except Exception as e_publish_general:
                 self.get_logger().error(f"Error general al publicar visualización: {e_publish_general}")

        except Exception as e_processing_callback:
            self.get_logger().error(f"Error general en procesamiento del timer_callback: {e_processing_callback}")
            traceback.print_exc()

    def cleanup_tello(self):
        """ Función de limpieza para detener stream y desconectar el Tello de forma segura. """
        self.get_logger().info("Iniciando limpieza de Tello...")
        if hasattr(self, 'stream_on') and self.stream_on:
            self.get_logger().info("Deteniendo stream de vídeo del Tello...")
            try:
                self.tello.streamoff()
            except Exception as e_streamoff:
                self.get_logger().error(f"Excepción durante tello.streamoff(): {e_streamoff}", throttle_duration_sec=5.0)
            self.stream_on = False # Marcar como apagado independientemente del resultado

        if hasattr(self, 'tello_connected') and self.tello_connected:
             # Opcional: Comprobar si está volando y advertir o intentar aterrizar (con precaución)
             # if hasattr(self.tello, 'is_flying') and self.tello.is_flying:
             #      self.get_logger().warn("El dron podría estar volando. NO se aterrizará automáticamente por este nodo.")
             #      # self.get_logger().info("Intentando aterrizar el dron...")
             #      # try: self.tello.land()
             #      # except Exception as e_land: self.get_logger().error(f"Excepción durante tello.land(): {e_land}")
             
             self.get_logger().info("Desconectando del Tello...")
             try:
                 self.tello.end() # Cierra la conexión con el Tello
             except Exception as e_end:
                 self.get_logger().error(f"Excepción durante tello.end(): {e_end}", throttle_duration_sec=5.0)
             self.tello_connected = False # Marcar como desconectado
        self.get_logger().info("Limpieza de Tello finalizada.")

    def destroy_node(self):
        """ Sobreescribir para asegurar limpieza al destruir el nodo. """
        self.get_logger().info("Cerrando el nodo detector de marcos rojos Tello (RGB)...")
        if hasattr(self, 'timer') and self.timer:
            self.timer.cancel() # Detener el temporizador
        
        self.cleanup_tello() # Llamar a la función de limpieza del Tello
        
        if hasattr(self, 'udp_socket') and self.udp_socket:
            self.udp_socket.close() # Cerrar el socket UDP
            self.get_logger().info("Socket UDP cerrado.")
        
        super().destroy_node() # Llamar al método destroy_node de la clase base

# --- Punto de Entrada Principal ---
def main(args=None):
    rclpy.init(args=args)
    node = None # Inicializar node a None
    try:
        node = TelloRedDetectorRGBPublisher() # Crear instancia del nodo
        if rclpy.ok(): # Comprobar si rclpy está bien antes de girar
            rclpy.spin(node)
    except KeyboardInterrupt:
        print("Ctrl+C detectado, cerrando nodo...")
    except SystemExit as e: # Para capturar SystemExit de __init__
        print(f"Saliendo debido a error de inicialización: {e}")
    except Exception as e_main:
        print(f"Error inesperado en main: {e_main}")
        traceback.print_exc()
    finally:
        print("Bloque finally en main: iniciando cierre...")
        if node is not None and rclpy.ok(): # Asegurarse que node existe y rclpy está ok
            print(f"Llamando a destroy_node() para {node.get_name()}...")
            try:
                 # La destrucción se maneja por el contexto de ROS2 al salir de spin o error,
                 # pero llamar a destroy_node explícitamente puede ser parte de una limpieza controlada.
                 # Sin embargo, rclpy.shutdown() se encargará de llamar a los destructores.
                 # No es estrictamente necesario llamarlo aquí si rclpy.shutdown() se ejecuta.
                 pass # node.destroy_node() # Esto se llamará automáticamente
            except Exception as e_destroy:
                 print(f"Error durante destroy_node explícito: {e_destroy}")
        
        if rclpy.ok(): # Comprobar si rclpy sigue activo antes de apagarlo
            print("Cerrando rclpy...")
            rclpy.shutdown()
        print("Programa ROS 2 finalizado.")

if __name__ == '__main__':
    main()