# Nodo ROS 2 que se conecta al Tello, detecta puertas rojas,
# calcula offset/distancia, envía datos por UDP 
# y PUBLICA la imagen con las detecciones en un topic ROS 2.

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
import math
import traceback
import threading

# --- Constantes ---
# Comunicación UDP (a moverDron_tello.py)
SERVER_IP = "127.0.0.1"
SERVER_PORT = 65432
# Topics ROS
ROS_TOPIC_DISPLAY_OUTPUT = '/tello/image_processed' # Topic para la imagen con detecciones
# Tello
TELLO_IP = '192.168.10.1'
# Bucle y Procesamiento
TIMER_PERIOD = 1.0 / 20.0 # Procesar a ~20 FPS (ajustar según capacidad)
FRAME_WIDTH_PROC = 640
FRAME_HEIGHT_PROC = 480
FRAMES_TIMEOUT_NONE = 600

# --- Constantes de Visión (¡¡¡ CALIBRAR / AJUSTAR ESTAS !!!) ---
TAMANO_REAL_PUERTA_M = 1.5 
DISTANCIA_FOCAL_PIXELS_TELLO = 920 # ¡CALIBRAR!
MIN_CONTOUR_AREA_TELLO = 2000      # ¡AJUSTAR!
# Rangos HSV para ROJO (¡¡AJUSTAR CON CALIBRADOR!!)
COLOR_LOWER_1 = np.array([0, 130, 90])
COLOR_UPPER_1 = np.array([10, 255, 255])
COLOR_LOWER_2 = np.array([160, 130, 90])
COLOR_UPPER_2 = np.array([179, 255, 255])
# --- Fin Constantes ---

def calcular_distancia(tamanio_aparente_pixels, distancia_focal_pixels, tamano_real_objeto_m):
    if tamanio_aparente_pixels <= 1: return float('inf')
    return (tamano_real_objeto_m * distancia_focal_pixels) / tamanio_aparente_pixels

class TelloGateDetectorPublisher(Node):
    def __init__(self):
        super().__init__('tello_gate_detector_publisher')
        self.get_logger().info("Iniciando Nodo Detector de Puertas y Publicador del Tello.")
        self.get_logger().info(f"Publicando visualización en: {ROS_TOPIC_DISPLAY_OUTPUT}")
        self.get_logger().info(f"Enviando datos de control a UDP {SERVER_IP}:{SERVER_PORT}")

        self.bridge = CvBridge()

        # Setup UDP Socket
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_address = (SERVER_IP, SERVER_PORT)

        # Setup Tello
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
            self.tello.streamoff(); time.sleep(0.5); self.tello.streamon()
            self.stream_on = True
            self.frame_reader = self.tello.get_frame_read()
            if self.frame_reader is None: raise RuntimeError("frame_reader es None.")
            self.get_logger().info("Stream activado y frame_reader obtenido.")
            time.sleep(1.5) # Pausa estabilización
        except Exception as e:
            self.get_logger().fatal(f"Error inicializando Tello: {e}")
            traceback.print_exc()
            if self.tello_connected: self.tello.end()
            rclpy.shutdown()
            raise SystemExit("Fallo en inicialización de Tello.")

        # Crear Publicador ROS para la imagen procesada
        qos_profile_display = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.display_publisher_ = self.create_publisher(Image, ROS_TOPIC_DISPLAY_OUTPUT, qos_profile_display)
        self.get_logger().info("Publicador ROS creado.")

        # Crear Temporizador ROS
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)
        self.get_logger().info(f"Temporizador creado con periodo: {TIMER_PERIOD:.3f}s")
        self.none_frame_streak = 0 # Mover contador aquí
        self.frame_counter_validos = 0 # Mover contador aquí


    def timer_callback(self):
        if self.frame_reader is None or not self.tello_connected: return
        if self.frame_reader.stopped:
            self.get_logger().error("Lector de frames detenido.")
            self.cleanup_tello(); rclpy.shutdown(); return

        current_frame_raw = self.frame_reader.frame

        if current_frame_raw is None:
            self.none_frame_streak += 1
            if self.none_frame_streak == 1 or self.none_frame_streak % 60 == 0: # Log menos frecuente
                 self.get_logger().warn(f"Frame es None (Streak: {self.none_frame_streak}). Lector vivo: {hasattr(self.frame_reader, 'thread') and self.frame_reader.thread.is_alive() if self.frame_reader.thread else 'N/A'}", throttle_duration_sec=5.0)
            if self.none_frame_streak > FRAMES_TIMEOUT_NONE:
                 self.get_logger().error(f"Timeout de frames None ({FRAMES_TIMEOUT_NONE}). Deteniendo nodo.")
                 self.cleanup_tello(); rclpy.shutdown(); return
            return # Salir del callback si no hay frame
        
        self.none_frame_streak = 0
        self.frame_counter_validos += 1

        try:
            # --- Procesamiento de Imagen ---
            img_proc = cv2.resize(current_frame_raw, (FRAME_WIDTH_PROC, FRAME_HEIGHT_PROC))
            img_display = img_proc.copy() # Para dibujar

            # 1. Segmentación Rojo
            img_hsv = cv2.cvtColor(img_proc, cv2.COLOR_BGR2HSV)
            mask1 = cv2.inRange(img_hsv, COLOR_LOWER_1, COLOR_UPPER_1)
            mask2 = cv2.inRange(img_hsv, COLOR_LOWER_2, COLOR_UPPER_2)
            final_mask = cv2.bitwise_or(mask1, mask2)
            # Opcional: Morfología
            # kernel = np.ones((3,3),np.uint8)
            # final_mask = cv2.morphologyEx(final_mask, cv2.MORPH_OPEN, kernel)
            # final_mask = cv2.morphologyEx(final_mask, cv2.MORPH_CLOSE, kernel)

            # 2. Contornos
            contours, _ = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # 3. Análisis de Contornos
            detected_targets_info = []
            frame_center_x = FRAME_WIDTH_PROC // 2
            frame_center_y = FRAME_HEIGHT_PROC // 2
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > MIN_CONTOUR_AREA_TELLO:
                    M = cv2.moments(cnt)
                    if M['m00'] > 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        offset_x = cx - frame_center_x
                        offset_y = cy - frame_center_y
                        rect = cv2.minAreaRect(cnt)
                        (box_w, box_h) = rect[1]
                        ancho_aparente_pixels = min(box_w, box_h) # O tu lógica preferida
                        distancia = calcular_distancia(ancho_aparente_pixels, DISTANCIA_FOCAL_PIXELS_TELLO, TAMANO_REAL_PUERTA_M)
                        detected_targets_info.append({'cx': cx, 'cy': cy, 'offset_x': offset_x, 'offset_y': offset_y,'distance': distancia, 'area': area, 'rect_obj': rect})

            # 4. Selección y Dibujo
            udp_offset_x, udp_offset_y, udp_distancia = 0.0, 0.0, float('inf')
            num_valid_targets = len(detected_targets_info)
            if num_valid_targets > 0:
                detected_targets_info.sort(key=lambda t: t['distance'])
                closest = detected_targets_info[0]
                udp_offset_x, udp_offset_y, udp_distancia = float(closest['offset_x']), float(closest['offset_y']), closest['distance']
                # Dibujar
                points = cv2.boxPoints(closest['rect_obj']); points = np.intp(points)
                cv2.drawContours(img_display, [points], -1, (0, 255, 0), 2)
                cv2.circle(img_display, (closest['cx'], closest['cy']), 5, (0, 255, 0), -1)
                cv2.putText(img_display, f"D:{closest['distance']:.1f}m O({closest['offset_x']},{closest['offset_y']})",
                            (closest['cx']-50, closest['cy']-15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

            # 5. Enviar UDP
            message = f"{udp_offset_x},{udp_offset_y},{udp_distancia:.2f},{num_valid_targets}"
            try:
                self.udp_socket.sendto(message.encode('utf-8'), self.server_address)
            except Exception as e_udp:
                 self.get_logger().error(f"Error enviando UDP: {e_udp}", throttle_duration_sec=5.0)

            # 6. Publicar Imagen Procesada en ROS
            try:
                vis_msg_out = self.bridge.cv2_to_imgmsg(img_display, encoding="bgr8")
                vis_msg_out.header.stamp = self.get_clock().now().to_msg()
                vis_msg_out.header.frame_id = "tello_camera_processed"
                self.display_publisher_.publish(vis_msg_out)
            except CvBridgeError as e_pub:
                 self.get_logger().error(f"Error CvBridge al publicar visualización: {e_pub}")
            except Exception as e_pub_gen:
                 self.get_logger().error(f"Error al publicar visualización: {e_pub_gen}")

        except Exception as e_proc:
            self.get_logger().error(f"Error general en procesamiento del callback: {e_proc}")
            traceback.print_exc()

    def cleanup_tello(self):
        """ Limpieza específica del Tello """
        self.get_logger().info("Iniciando limpieza de Tello...")
        if hasattr(self, 'stream_on') and self.stream_on:
            self.get_logger().info("Deteniendo stream...")
            try: self.tello.streamoff()
            except Exception as e: self.get_logger().error(f"Excepción en streamoff: {e}", throttle_duration_sec=5.0)
            self.stream_on = False
        if hasattr(self, 'tello_connected') and self.tello_connected:
             if hasattr(self.tello, 'is_flying') and self.tello.is_flying:
                  self.get_logger().warn("El dron podría estar volando. Este nodo NO lo aterrizará.")
             self.get_logger().info("Desconectando del Tello...")
             try: self.tello.end()
             except Exception as e: self.get_logger().error(f"Excepción en end: {e}", throttle_duration_sec=5.0)
             self.tello_connected = False
        self.get_logger().info("Limpieza de Tello finalizada.")

    def destroy_node(self):
        self.get_logger().info("Cerrando nodo detector de puertas Tello...")
        if hasattr(self, 'timer') and self.timer: self.timer.cancel()
        self.cleanup_tello()
        if hasattr(self, 'udp_socket') and self.udp_socket:
            self.udp_socket.close()
            self.get_logger().info("Socket UDP cerrado.")
        super().destroy_node()

# --- Main (similar al anterior) ---
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TelloGateDetectorPublisher()
        if rclpy.ok():
            rclpy.spin(node)
    except KeyboardInterrupt:
        print("Ctrl+C detectado, cerrando nodo...")
    except SystemExit as e:
        print(f"Saliendo debido a error de inicialización: {e}")
    except Exception as e:
        print(f"Error inesperado en main: {e}")
        traceback.print_exc()
    finally:
        print("Bloque finally en main...")
        # La destrucción debería manejarse automáticamente por rclpy al salir de spin o error
        # Pero podemos asegurar el shutdown de rclpy
        if rclpy.ok():
            print("Cerrando rclpy...")
            rclpy.shutdown()
        print("Programa finalizado.")

if __name__ == '__main__':
    main()