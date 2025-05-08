"""
tratarImagen.py
Se suscribe a un topic de imagen ROS 2, aplica MiDaS para estimar profundidad,
detecta "puertas" basándose en la profundidad y navega hacia la más cercana.
Envía información de la puerta más cercana por UDP.

Dependencias:
sudo apt install libgtk2.0-dev pkg-config
pip install opencv-python
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import torch
import numpy as np
import socket

SERVER_IP = "127.0.0.1"
SERVER_PORT = 65432
UMBRAL_PROFUNDIDAD_PUERTA_LEJANA = 1.5  # Metros: Umbral para considerar una región como posible "puerta lejana"
UMBRAL_PROFUNDIDAD_PUERTA_CERCANA = 0.5 # Metros: Umbral para descartar regiones demasiado cercanas
MIN_AREA_PUERTA_PIXELS = 500       # Área mínima en píxeles para considerar una región como puerta
FACTOR_EXPANSION_ROI = 1.2        # Factor para expandir la ROI alrededor de la posible puerta

class MidasPuertaNavigator(Node):
    def __init__(self):
        super().__init__('midas_puerta_navigator')
        self.subscription = self.create_subscription(
            Image,
            '/world/puertas3/model/x500_mono_cam_0/link/camera_link/sensor/imager/image',
            self.listener_callback,
            10)
        self.br = CvBridge()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_address = (SERVER_IP, SERVER_PORT)
        self.get_logger().info(f"Navegador MiDaS Puertas: Enviando a UDP {SERVER_IP}:{SERVER_PORT}")

        # Cargar modelo MiDaS y transformaciones una sola vez
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"Cargando modelo MiDaS (MiDaS_small) en {self.device}...")
        try:
            self.midas = torch.hub.load("intel-isl/MiDaS", "MiDaS_small")
            self.midas.to(self.device)
            self.midas.eval()
            midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
            self.transform = midas_transforms.small_transform
            self.get_logger().info("Modelo MiDaS y transformaciones cargadas.")
        except Exception as e:
            self.get_logger().fatal(f"Error cargando modelo MiDaS: {e}. Asegúrate de tener conexión a internet y la librería 'timm' instalada (`pip install timm`).")
            rclpy.shutdown()
            return

    def listener_callback(self, msg):
        try:
            cv_image_bgr = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image_rgb = cv2.cvtColor(cv_image_bgr, cv2.COLOR_BGR2RGB)
        except CvBridgeError as e:
            self.get_logger().error(f"Error CvBridge: {e}")
            return

        try:
            # Preprocesar imagen para MiDaS
            input_batch = self.transform(cv_image_rgb).to(self.device)

            with torch.no_grad():
                prediction = self.midas(input_batch)
                prediction = torch.nn.functional.interpolate(
                    prediction.unsqueeze(1),
                    size=cv_image_rgb.shape[:2],
                    mode="bicubic",
                    align_corners=False,
                ).squeeze()

            depth_map = prediction.cpu().numpy()

            # --- Detección de "Puertas" basada en Profundidad ---
            puertas_detectadas = []
            altura, anchura = depth_map.shape

            # Aplicar umbrales de profundidad para identificar posibles regiones de "puerta"
            mascara_puertas = (depth_map > UMBRAL_PROFUNDIDAD_PUERTA_CERCANA) & (depth_map < UMBRAL_PROFUNDIDAD_PUERTA_LEJANA)
            mascara_puertas = mascara_puertas.astype(np.uint8) * 255

            # Encontrar contornos en la máscara de posibles puertas
            contornos_puertas, _ = cv2.findContours(mascara_puertas, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contorno in contornos_puertas:
                area = cv2.contourArea(contorno)
                if area > MIN_AREA_PUERTA_PIXELS:
                    x, y, w, h = cv2.boundingRect(contorno)
                    centro_x = x + w // 2
                    centro_y = y + h // 2
                    distancia_promedio = np.mean(depth_map[y:y+h, x:x+w]) if w > 0 and h > 0 else float('inf')

                    puertas_detectadas.append({
                        'bbox': (x, y, w, h),
                        'centro': (centro_x, centro_y),
                        'distancia': distancia_promedio,
                        'area': area
                    })

            # Encontrar la puerta más cercana
            puerta_mas_cercana = None
            if puertas_detectadas:
                puertas_detectadas.sort(key=lambda p: p['distancia'])
                puerta_mas_cercana = puertas_detectadas[0]

            # --- Enviar información por UDP ---
            offset_x_puerta, offset_y_puerta, distancia_puerta = 0, 0, 0.0
            num_puertas_detectadas = len(puertas_detectadas)

            if puerta_mas_cercana:
                centro_x_imagen = anchura // 2
                centro_y_imagen = altura // 2
                offset_x_puerta = puerta_mas_cercana['centro'][0] - centro_x_imagen
                offset_y_puerta = puerta_mas_cercana['centro'][1] - centro_y_imagen
                distancia_puerta = puerta_mas_cercana['distancia']

            message = f"{offset_x_puerta},{offset_y_puerta},{distancia_puerta:.2f},{num_puertas_detectadas}"
            try:
                self.sock.sendto(message.encode('utf-8'), self.server_address)
            except Exception as e:
                self.get_logger().error(f"Error enviando UDP: {e}")

            # --- Visualización (opcional) ---
            img_display_depth = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            img_display_depth = cv2.cvtColor(img_display_depth, cv2.COLOR_GRAY2BGR)

            for puerta in puertas_detectadas:
                x, y, w, h = puerta['bbox']
                color = (0, 255, 0) # Verde para todas las puertas
                if puerta == puerta_mas_cercana:
                    color = (255, 0, 0) # Azul para la más cercana
                    cv2.rectangle(img_display_depth, (x, y), (x + w, y + h), color, 2)
                    cv2.circle(img_display_depth, puerta['centro'], 5, color, -1)
                    info_text = f"D:{puerta['distancia']:.2f}m"
                    cv2.putText(img_display_depth, info_text, (puerta['centro'][0] + 10, puerta['centro'][1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                else:
                    cv2.rectangle(img_display_depth, (x, y), (x + w, y + h), (0, 150, 0), 1) # Verde más oscuro para otras

            cv2.imshow("Mapa de Profundidad (Puertas)", img_display_depth)
            cv2.waitKey(1)

        except CvBridgeError as e:
            self.get_logger().error(f"Error de CvBridge: {e}")
        except Exception as e:
            self.get_logger().error(f"Error durante el procesamiento: {e}")

    def destroy_node(self):
        self.get_logger().info('Cerrando nodo navegador de puertas MiDaS...')
        if hasattr(self, 'sock') and self.sock:
            self.sock.close()
            self.get_logger().info('Socket UDP cerrado.')
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    midas_puerta_navigator = None
    try:
        midas_puerta_navigator = MidasPuertaNavigator()
        rclpy.spin(midas_puerta_navigator)
    except KeyboardInterrupt:
        print("Ctrl+C detectado, cerrando nodo...")
    finally:
        if midas_puerta_navigator:
            midas_puerta_navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Programa finalizado.")

if __name__ == '__main__':
    main()