import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import traceback

ROS_TOPIC_IMAGEN_RAW_INPUT = '/tello/imagen_raw'
ROS_TOPIC_IMAGEN_PROCESADA_OUTPUT = '/tello/imagen_procesada'
ROS_TOPIC_DATOS_DETECCION_OUTPUT = '/tello/datos_deteccion'

FRAME_WIDTH_PROC = 640
FRAME_HEIGHT_PROC = 480

TAMANO_REAL_PUERTA_M = 0.35      # Tamaño real del objeto en metros (altura o ancho)
DISTANCIA_FOCAL_PIXELS_TELLO = 920
MIN_CONTOUR_AREA_TELLO = 200
"""
# Rangos HSV para ROJO 
# El rojo a menudo cruza el límite 0/179 en el espacio HUE de OpenCV
COLOR_LOWER_1 = np.array([0, 130, 90])
COLOR_UPPER_1 = np.array([10, 255, 255])
COLOR_LOWER_2 = np.array([160, 130, 90])
COLOR_UPPER_2 = np.array([179, 255, 255])
"""
#Rangos para el azul
COLOR_LOWER_1 = np.array([60, 50, 50])    
COLOR_UPPER_1 = np.array([120, 255, 255]) 


def calcular_distancia(tamanio_aparente_pixels, distancia_focal_pixels, tamano_real_objeto_m):
    if tamanio_aparente_pixels <= 1:
        return float('inf')
    return (tamano_real_objeto_m * distancia_focal_pixels) / tamanio_aparente_pixels

class NodoImagen(Node):
    def __init__(self):
        super().__init__('nodo_imagen_imagen')
        self.get_logger().info("Iniciando Nodo Procesador de Imagen.")
        self.bridge = CvBridge()

        # Suscriptor para la imagen RAW
        qos_profile_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1 # Solo procesar el último frame recibido
        )
        self.suscripcion_imagen = self.create_subscription(
            Image,
            ROS_TOPIC_IMAGEN_RAW_INPUT,
            self.callback_procesamiento_imagen,
            qos_profile_sub
        )
        self.get_logger().info(f"Suscrito a imagen RAW en: {ROS_TOPIC_IMAGEN_RAW_INPUT}")

        # Publicador para la imagen procesada
        qos_profile_pub_img = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.publicador_imagen_procesada = self.create_publisher(
            Image, ROS_TOPIC_IMAGEN_PROCESADA_OUTPUT, qos_profile_pub_img)
        self.get_logger().info(f"Publicando imagen procesada en: {ROS_TOPIC_IMAGEN_PROCESADA_OUTPUT}")

        # Publicador para los datos de detección
        qos_profile_pub_data = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.publicador_datos_deteccion = self.create_publisher(
            Float32MultiArray, ROS_TOPIC_DATOS_DETECCION_OUTPUT, qos_profile_pub_data)
        self.get_logger().info(f"Publicando datos de detección en: {ROS_TOPIC_DATOS_DETECCION_OUTPUT}")

    def callback_procesamiento_imagen(self, msg_imagen_ros):
        try:
            # Convertir mensaje ROS Image (esperamos BGR8 desde nodo_camara_tello) a frame OpenCV
            current_frame_bgr_raw = self.bridge.imgmsg_to_cv2(msg_imagen_ros, desired_encoding="bgr8")
        except CvBridgeError as e_bridge:
            self.get_logger().error(f"Error CvBridge al convertir imagen RAW: {e_bridge}")
            return
        except Exception as e_conversion:
            self.get_logger().error(f"Error general convirtiendo imagen RAW: {e_conversion}")
            return

        if current_frame_bgr_raw is None:
            self.get_logger().warn("Frame BGR recibido es None después de conversión.", throttle_duration_sec=5.0)
            return

        try:
            #Inicio Lógica de Procesamiento
            img_proc_bgr = cv2.resize(current_frame_bgr_raw, (FRAME_WIDTH_PROC, FRAME_HEIGHT_PROC))
            img_proc = cv2.cvtColor(img_proc_bgr, cv2.COLOR_BGR2RGB)

            # 1. Segmentación de Color Rojo usando HSV
            img_hsv = cv2.cvtColor(img_proc, cv2.COLOR_BGR2HSV)
            final_mask_blue = cv2.inRange(img_hsv, COLOR_LOWER_1, COLOR_UPPER_1)
            final_mask_red = final_mask_blue

            #mask1_red = cv2.inRange(img_hsv, COLOR_LOWER_1, COLOR_UPPER_1)
            #mask2_red = cv2.inRange(img_hsv, COLOR_LOWER_2, COLOR_UPPER_2)
            #final_mask_red = cv2.bitwise_or(mask1_red, mask2_red)

            # 2. Encontrar Contornos
            contours, _ = cv2.findContours(final_mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # 3. Análisis de Contornos
            detected_targets_info = []
            frame_center_x_proc = FRAME_WIDTH_PROC // 2
            frame_center_y_proc = FRAME_HEIGHT_PROC // 2

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > MIN_CONTOUR_AREA_TELLO:
                    M = cv2.moments(cnt)
                    if M['m00'] > 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        offset_x_pixels = cx - frame_center_x_proc
                        offset_y_pixels = cy - frame_center_y_proc # Positivo es abajo, negativo es arriba

                        rect_min_area = cv2.minAreaRect(cnt)
                        (box_w, box_h) = rect_min_area[1]
                        tamanio_aparente_pixels_para_dist = max(box_w, box_h)

                        distancia_m_calculada = calcular_distancia(tamanio_aparente_pixels_para_dist,
                                                              DISTANCIA_FOCAL_PIXELS_TELLO,
                                                              TAMANO_REAL_PUERTA_M)
                        detected_targets_info.append({
                            'cx': cx, 'cy': cy,
                            'offset_x_pixels': offset_x_pixels,
                            'offset_y_pixels': offset_y_pixels,
                            'distance': distancia_m_calculada,
                            'area': area,
                            'rect_obj': rect_min_area
                        })

            # 4. Selección del Mejor Objetivo y Preparación de Datos
            offset_x_norm_publicar, offset_y_norm_publicar, distancia_m_publicar = 0.0, 0.0, float('inf')
            num_valid_targets_publicar = len(detected_targets_info)

            if num_valid_targets_publicar > 0:
                detected_targets_info.sort(key=lambda t: t['distance']) # Ordenar por distancia (más cercano primero)
                closest_target = detected_targets_info[0]

                offset_x_norm_publicar = closest_target['offset_x_pixels'] / (FRAME_WIDTH_PROC / 2.0)
                # Normalizar offset_y: positivo es ABAJO en imagen, negativo es ARRIBA en imagen
                # El nodo de control debe interpretar esto correctamente para el movimiento del dron
                # (ej. offset_y_norm positivo -> dron debe subir)
                offset_y_norm_publicar = closest_target['offset_y_pixels'] / (FRAME_HEIGHT_PROC / 2.0)
                distancia_m_publicar = closest_target['distance']

                # Dibujar sobre img_con_dibujos_bgr
                points = cv2.boxPoints(closest_target['rect_obj'])
                points = np.intp(points)
                cv2.drawContours(img_proc, [points], -1, (0, 255, 0), 2) # Verde
                cv2.circle(img_proc, (closest_target['cx'], closest_target['cy']), 5, (0, 0, 255), -1) # Rojo

                info_text = f"D:{distancia_m_publicar:.1f}m X:{offset_x_norm_publicar:.2f} Y:{offset_y_norm_publicar:.2f}"
                cv2.putText(img_proc, info_text,
                            (closest_target['cx'] - 70, closest_target['cy'] - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA) # Amarillo
            #Fin Lógica de Procesamiento

            # 5. Publicar Datos de Detección
            msg_datos_deteccion = Float32MultiArray()
            # Formato: [offset_x_normalizado, offset_y_normalizado, distancia_metros, numero_objetivos_validos]
            msg_datos_deteccion.data = [
                offset_x_norm_publicar,
                offset_y_norm_publicar,
                distancia_m_publicar,
                float(num_valid_targets_publicar) # Asegurar que sea float
            ]
            self.publicador_datos_deteccion.publish(msg_datos_deteccion)

            # 6. Publicar Imagen Procesada (convertida a RGB8 para visualizadores estándar)
            try:
                img_publish_rgb = cv2.cvtColor(img_proc, cv2.COLOR_BGR2RGB)
                ros_image_msg_out = self.bridge.cv2_to_imgmsg(img_publish_rgb, encoding="rgb8")
                ros_image_msg_out.header.stamp = msg_imagen_ros.header.stamp # Usar el mismo timestamp
                ros_image_msg_out.header.frame_id = "tello_camera_processed_rgb"
                self.publicador_imagen_procesada.publish(ros_image_msg_out)
            except CvBridgeError as e_cv_bridge_pub:
                self.get_logger().error(f"Error CvBridge al convertir/publicar imagen procesada: {e_cv_bridge_pub}")
            except Exception as e_publish_general:
                self.get_logger().error(f"Error general al publicar visualización: {e_publish_general}")

        except Exception as e_processing_callback:
            self.get_logger().error(f"Error general en procesamiento del callback_procesamiento_imagen: {e_processing_callback}")
            self.get_logger().error(traceback.format_exc())

    def destroy_node(self):
        self.get_logger().info("Destruyendo NodoImagen...")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    nodo_imagen = None
    try:
        nodo_imagen = NodoImagen()
        rclpy.spin(nodo_imagen)
    except KeyboardInterrupt:
        if nodo_imagen:
            nodo_imagen.get_logger().info("Ctrl+C detectado, cerrando NodoImagen.")
        else:
            print("Ctrl+C detectado antes de inicializar NodoImagen.")
    except Exception as e_main:
        if nodo_imagen:
            nodo_imagen.get_logger().fatal(f"Error inesperado en main de NodoImagen: {e_main}")
            nodo_imagen.get_logger().fatal(traceback.format_exc())
        else:
            print(f"Error inesperado en main antes de inicializar NodoImagen: {e_main}")
            print(traceback.format_exc())
    finally:
        if nodo_imagen:
            nodo_imagen.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Programa NodoImagen finalizado.")

if __name__ == '__main__':
    main()
