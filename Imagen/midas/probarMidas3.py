#Escribir en CSV

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import torch
import numpy as np
import csv
from datetime import datetime

CAMARA_TOPIC = '/world/default/model/x500_mono_cam_0/link/camera_link/sensor/imager/image'
MODEL_TYPE = "MiDaS_small"
COLORMAP_SELECCIONADO = cv2.COLORMAP_INFERNO
NOMBRE_ARCHIVO_CSV = 'distancias_normalizadas.csv'

class MidasVisualizerNodeCV_CSV(Node):
    def __init__(self):
        super().__init__('midas_visualizer_node_cv_csv')
        self.get_logger().info(f"Iniciando Nodo Visualizador MiDaS (OpenCV) con CSV para topic: {CAMARA_TOPIC}")

        self.subscription = self.create_subscription(
            Image,
            CAMARA_TOPIC,
            self.listener_callback,
            10)
        self.subscription
        self.bridge = CvBridge()

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

        cv2.namedWindow("Mapa de Profundidad MiDaS (OpenCV)", cv2.WINDOW_AUTOSIZE)

        self.archivo_csv = open(NOMBRE_ARCHIVO_CSV, 'w', newline='')
        self.escritor_csv = csv.writer(self.archivo_csv)
        self.primera_imagen = True

    def listener_callback(self, msg):
        self.get_logger().info(f'Recibiendo frame: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}', throttle_duration_sec=1.0)
        try:
            cv_image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image_rgb = cv2.cvtColor(cv_image_bgr, cv2.COLOR_BGR2RGB)
        except CvBridgeError as e:
            self.get_logger().error(f"Error CvBridge: {e}")
            return
        except Exception as e:
             self.get_logger().error(f"Error convirtiendo imagen: {e}")
             return

        try:
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

            depth_min = np.min(depth_map)
            depth_max = np.max(depth_map)
            depth_normalized = (depth_map - depth_min) / (depth_max - depth_min)

            if not self.primera_imagen:
                self.escritor_csv.writerow(['-' * 20])
            else:
                self.primera_imagen = False
            self.escritor_csv.writerow(depth_normalized.flatten().tolist())
            self.archivo_csv.flush()

            depth_map_normalized_vis = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

            depth_heatmap_bgr = cv2.applyColorMap(depth_map_normalized_vis, COLORMAP_SELECCIONADO)

            cv2.imshow("Mapa de Profundidad MiDaS (OpenCV)", depth_heatmap_bgr)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error durante procesamiento MiDaS o visualización OpenCV: {e}")

    def destroy_node(self):
        self.get_logger().info("Cerrando nodo visualizador MiDaS (OpenCV) y archivo CSV...")
        cv2.destroyAllWindows()
        self.archivo_csv.close()
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