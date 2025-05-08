"""
probarMidas2.py
Se suscribe a un topic de imagen ROS 2, aplica MiDaS para estimar profundidad,
muestra el mapa de profundidad coloreado en tiempo real usando OpenCV y
escribe los valores normalizados de la distancia en un archivo CSV.

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
import csv
from datetime import datetime

# --- Configuración ---
CAMARA_TOPIC = '/world/puertas3/model/x500_mono_cam_0/link/camera_link/sensor/imager/image'
# Modelo MiDaS a usar ("MiDaS_small", "MiDaS", "DPT_Large", "DPT_Hybrid")
MODEL_TYPE = "MiDaS_small"
# Mapa de color de OpenCV a usar (ver opciones en la documentación de OpenCV: COLORMAP_...)
COLORMAP_SELECCIONADO = cv2.COLORMAP_INFERNO
# Nombre del archivo CSV para guardar los datos
NOMBRE_ARCHIVO_CSV = 'distancias_normalizadas.csv'

class MidasVisualizerNodeCV_CSV(Node):
    def __init__(self):
        super().__init__('midas_visualizer_node_cv_csv')
        self.get_logger().info(f"Iniciando Nodo Visualizador MiDaS (OpenCV) con CSV para topic: {CAMARA_TOPIC}")

        # Suscripción al topic de imagen
        self.subscription = self.create_subscription(
            Image,
            CAMARA_TOPIC,
            self.listener_callback,
            10) # QoS profile depth
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

        # Cargar modelo MiDaS y transformaciones una sola vez
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"Cargando modelo MiDaS ({MODEL_TYPE}) en {self.device}...")
        try:
            self.midas = torch.hub.load("intel-isl/MiDaS", MODEL_TYPE)
            self.midas.to(self.device)
            self.midas.eval()

            midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
            self.transform = midas_transforms.small_transform if MODEL_TYPE == "MiDaS_small" else midas_transforms.dpt_transform
            self.get_logger().info("Modelo MiDaS y transformaciones cargadas.")
        except Exception as e:
            self.get_logger().fatal(f"Error cargando modelo MiDaS: {e}. Asegúrate de tener conexión a internet y la librería 'timm' instalada (`pip install timm`).")
            rclpy.shutdown()
            return

        # Crear ventana de OpenCV
        cv2.namedWindow("Mapa de Profundidad MiDaS (OpenCV)", cv2.WINDOW_AUTOSIZE)

        # Inicializar el archivo CSV
        self.archivo_csv = open(NOMBRE_ARCHIVO_CSV, 'w', newline='')
        self.escritor_csv = csv.writer(self.archivo_csv)
        self.primera_imagen = True

    def listener_callback(self, msg):
        self.get_logger().info(f'Recibiendo frame: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}', throttle_duration_sec=1.0)
        try:
            # Convertir mensaje ROS Image a imagen OpenCV (BGR)
            cv_image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Convertir a RGB para MiDaS
            cv_image_rgb = cv2.cvtColor(cv_image_bgr, cv2.COLOR_BGR2RGB)
        except CvBridgeError as e:
            self.get_logger().error(f"Error CvBridge: {e}")
            return
        except Exception as e:
             self.get_logger().error(f"Error convirtiendo imagen: {e}")
             return

        try:
            # Preprocesar imagen para MiDaS
            input_batch = self.transform(cv_image_rgb).to(self.device)

            # Realizar inferencia
            with torch.no_grad():
                prediction = self.midas(input_batch)
                prediction = torch.nn.functional.interpolate(
                    prediction.unsqueeze(1),
                    size=cv_image_rgb.shape[:2],
                    mode="bicubic",
                    align_corners=False,
                ).squeeze()

            # Mover a CPU y convertir a NumPy
            depth_map = prediction.cpu().numpy()

            # Normalizar el mapa de profundidad al rango 0-1
            depth_min = np.min(depth_map)
            depth_max = np.max(depth_map)
            depth_normalized = (depth_map - depth_min) / (depth_max - depth_min)

            # Escribir los valores normalizados en el CSV
            if not self.primera_imagen:
                self.escritor_csv.writerow(['-' * 20]) # Separador entre imágenes
            else:
                self.primera_imagen = False
            self.escritor_csv.writerow(depth_normalized.flatten().tolist())
            self.archivo_csv.flush() # Asegurar que los datos se escriban inmediatamente

            # --- Visualización con OpenCV ---
            # 1. Normalizar el mapa de profundidad al rango 0-255 (para 8 bits)
            depth_map_normalized_vis = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

            # 2. Aplicar un mapa de color para visualizar
            depth_heatmap_bgr = cv2.applyColorMap(depth_map_normalized_vis, COLORMAP_SELECCIONADO)

            # 3. Mostrar la imagen resultante en la ventana de OpenCV
            cv2.imshow("Mapa de Profundidad MiDaS (OpenCV)", depth_heatmap_bgr)
            cv2.waitKey(1) # Muy importante para que la ventana se refresque

        except Exception as e:
            self.get_logger().error(f"Error durante procesamiento MiDaS o visualización OpenCV: {e}")

    def destroy_node(self):
        self.get_logger().info("Cerrando nodo visualizador MiDaS (OpenCV) y archivo CSV...")
        cv2.destroyAllWindows() # Cerrar ventanas de OpenCV
        self.archivo_csv.close() # Cerrar el archivo CSV
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    midas_visualizer_node_cv_csv = None
    try:
        midas_visualizer_node_cv_csv = MidasVisualizerNodeCV_CSV()
        rclpy.spin(midas_visualizer_node_cv_csv)
    except KeyboardInterrupt:
        print("Ctrl+C detectado, cerrando nodo...")
    except Exception as e:
        if midas_visualizer_node_cv_csv:
            midas_visualizer_node_cv_csv.get_logger().fatal(f"Error inesperado: {e}")
        else:
            print(f"Error durante inicialización: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if midas_visualizer_node_cv_csv:
            midas_visualizer_node_cv_csv.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Programa finalizado.")

if __name__ == '__main__':
    main()