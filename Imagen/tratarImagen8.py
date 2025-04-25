"""
Exactamente como tratarImagen7.py, pero añadiendo las
  comprobaciones si el centroide cae sobre un color rojo
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import socket
import math

SERVER_IP = "127.0.0.1"
SERVER_PORT = 65432
TAMANO_REAL_PUERTA_M = 1.5     # Tamaño real (ancho) de la puerta en metros para estimar distancia
DISTANCIA_FOCAL_PIXELS = 539.35 # Distancia focal de la cámara (en píxeles)
MIN_CONTOUR_AREA = 150        # Área mínima en píxeles para considerar un contorno como posible puerta
AREA_UMBRAL_GRANDE = 20     # Área en píxeles a partir de la cual se aplica el chequeo del centroide

# Rango de color BGR para detectar el rojo (AJUSTAR según tu simulación)
COLOR_LOWER = np.array([0, 0, 100]) # Límite inferior BGR
COLOR_UPPER = np.array([80, 80, 255]) # Límite superior BGR

class ImageProcessorSimple(Node):
    def __init__(self):
        super().__init__('image_processor_simple')
        self.subscription = self.create_subscription(
            Image,
            '/world/puertas2/model/x500_mono_cam_0/link/camera_link/sensor/imager/image',
            self.listener_callback,
            10)
        self.br = CvBridge()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_address = (SERVER_IP, SERVER_PORT)
        self.get_logger().info(f"Procesador Simple: Enviando a UDP {SERVER_IP}:{SERVER_PORT}")

    def calcular_distancia(self, tamanio_ancho_pixels):
        # Calcula distancia basada en el ancho en píxeles y el tamaño real conocido
        if tamanio_ancho_pixels <= 1:
            return float('inf') # Evita división por cero o valores inválidos
        return (TAMANO_REAL_PUERTA_M * DISTANCIA_FOCAL_PIXELS) / tamanio_ancho_pixels

    def listener_callback(self, data):
        try:
            # Convierte mensaje ROS a imagen OpenCV (BGR por defecto)
            current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error CvBridge: {e}")
            return

        img_display = current_frame.copy() 
        height, width, _ = img_display.shape
        center_x = width // 2
        center_y = height // 2

        # Segmentación por color para obtener máscara binaria
        mask = cv2.inRange(current_frame, COLOR_LOWER, COLOR_UPPER)
        # Encontrar contornos en la máscara
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        detected_targets = [] 
        for contour in contours:
            area = cv2.contourArea(contour) # area del contorno
            if area > MIN_CONTOUR_AREA:
                M = cv2.moments(contour) 
                if M['m00'] > 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

                    if area > AREA_UMBRAL_GRANDE:
                        # Solo aplicar si el área supera el umbral grande
                        color_centroide = current_frame[cY, cX]
                        # Comprobar si el color del centroide está en el rango rojo definido
                        es_rojo = np.all(color_centroide >= COLOR_LOWER) and np.all(color_centroide <= COLOR_UPPER)

                        if es_rojo:
                            # Si el centroide cae sobre un píxel rojo, probablemente solo vemos el marco
                            self.get_logger().warn(f"Detección grande (área={area:.0f}) con centroide en rojo ({color_centroide}). Probablemente poste. Descartando.")
                            continue # Saltar al siguiente contorno, no añadir este a detected_targets

                    #Continuar con lo habitual;
                    # Calcular rectángulo de área mínima y ancho en píxeles
                    rect = cv2.minAreaRect(contour)
                    (w_rot, h_rot) = rect[1]
                    tamanio_ancho_pixels = float(min(w_rot, h_rot))

                    distancia_estimada = self.calcular_distancia(tamanio_ancho_pixels)
                    offset_x = cX - center_x
                    offset_y = cY - center_y

                    detected_targets.append({
                        'offset_x': offset_x, 'offset_y': offset_y,
                        'distance': distancia_estimada, 'centroide': (cX, cY),
                        'rect': rect
                        # si se neceista area añadir qaui
                    })

        target_offset_x, target_offset_y, target_distance = 0, 0, 0.0
        num_targets_detected = len(detected_targets)
        closest_target = None

        if num_targets_detected > 0:
            detected_targets.sort(key=lambda t: t['distance'])
            closest_target = detected_targets[0] # El primero es el más cercano
            target_offset_x = closest_target['offset_x']
            target_offset_y = closest_target['offset_y']
            target_distance = closest_target['distance']

        message = f"{target_offset_x},{target_offset_y},{target_distance:.2f},{num_targets_detected}"
        try:
            self.sock.sendto(message.encode('utf-8'), self.server_address)
        except Exception as e:
            self.get_logger().error(f"Error enviando UDP: {e}")

        for i, target in enumerate(detected_targets):
            color = (0, 255, 0) # Verde por defecto
            if target == closest_target: color = (255, 0, 0) # Azul para el más cercano
            box_points = cv2.boxPoints(target['rect'])
            cv2.drawContours(img_display, [np.intp(box_points)], 0, color, 2)
            cv2.circle(img_display, target['centroide'], 5, color, -1)
            info_text = f"D:{target['distance']:.1f}"
            cv2.putText(img_display, info_text, (target['centroide'][0] + 10, target['centroide'][1]),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        cv2.imshow("Deteccion Simple", img_display)
        cv2.waitKey(1) 

    def destroy_node(self):
        self.get_logger().info('Limpiando nodo procesador...')
        if hasattr(self, 'sock') and self.sock:
             self.sock.close()
             self.get_logger().info('Socket UDP cerrado.')
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    image_processor = None
    try:
        image_processor = ImageProcessorSimple()
        rclpy.spin(image_processor)
    except KeyboardInterrupt:
        print("--> Ctrl+C detectado.")
    finally:
        print("--> Limpiando...")
        if image_processor:
             image_processor.destroy_node()
        if rclpy.ok():
             rclpy.shutdown()
        print("--> Programa finalizado.")

if __name__ == '__main__':
    main()