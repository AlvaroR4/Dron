# midas_publisher_node.py
# Se suscribe a un topic de imagen ROS 2, aplica MiDaS para estimar profundidad
# y PUBLICA el mapa de profundidad coloreado en un nuevo topic ROS 2.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy # Para definir QoS del publicador
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import torch
import numpy as np
import traceback

# --- Configuración ---
CAMARA_TOPIC_ENTRADA = '/world/default/model/x500_mono_cam_0/link/camera_link/sensor/imager/image'
DEPTH_MAP_TOPIC_SALIDA = '/midas/depth_display' # Nombre del topic donde publicaremos el resultado

MODEL_TYPE = "MiDaS_small"
COLORMAP_SELECCIONADO = cv2.COLORMAP_INFERNO

class MidasPublisherNode(Node):
    def __init__(self):
        # Cambiamos el nombre del nodo para reflejar su función
        super().__init__('midas_publisher_node') 
        self.get_logger().info(f"Iniciando Nodo Publicador MiDaS.")
        self.get_logger().info(f"Suscribiendo a: {CAMARA_TOPIC_ENTRADA}")
        self.get_logger().info(f"Publicando en: {DEPTH_MAP_TOPIC_SALIDA}")

        # Suscripción al topic de imagen original
        self.subscription = self.create_subscription(
            Image,
            CAMARA_TOPIC_ENTRADA,
            self.listener_callback,
            10) # QoS depth
        self.subscription
        self.bridge = CvBridge()

        # Cargar modelo MiDaS (igual que antes)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"Cargando modelo MiDaS ({MODEL_TYPE}) en {self.device}...")
        try:
            self.midas = torch.hub.load("intel-isl/MiDaS", MODEL_TYPE, trust_repo=True)
            self.midas.to(self.device)
            self.midas.eval()

            midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms", trust_repo=True)
            self.transform = midas_transforms.small_transform if MODEL_TYPE == "MiDaS_small" else midas_transforms.dpt_transform
            self.get_logger().info("Modelo MiDaS y transformaciones cargadas.")
        except Exception as e:
            self.get_logger().fatal(f"Error cargando modelo MiDaS: {e}.")
            traceback.print_exc()
            rclpy.shutdown()
            raise SystemExit(f"Fallo al cargar modelo MiDaS: {e}")

        # --- Crear el Publicador para la Imagen Procesada ---
        # Definir un perfil QoS adecuado para imágenes/vídeo (puede variar según necesidad)
        qos_profile_publisher = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # UDP suele ser BEST_EFFORT
            history=HistoryPolicy.KEEP_LAST,       # Solo nos interesa el último frame
            depth=5                                # Buffer pequeño
        )
        self.publisher_ = self.create_publisher(
            Image,                  # Tipo de mensaje a publicar
            DEPTH_MAP_TOPIC_SALIDA, # Nombre del topic de salida
            qos_profile_publisher   # Perfil QoS
        )
        self.get_logger().info(f"Publicador creado en '{DEPTH_MAP_TOPIC_SALIDA}'.")
        
        # Ya NO creamos ventana con cv2.namedWindow
        self.get_logger().info("Inicialización completa. Esperando mensajes ROS...")


    def listener_callback(self, msg):
        self.get_logger().info(f'Callback: Recibido frame {msg.header.stamp.sec}.{msg.header.stamp.nanosec}', throttle_duration_sec=2.0) # Log menos frecuente
        try:
            # Convertir mensaje ROS Image a imagen OpenCV (BGR)
            cv_image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Convertir a RGB para MiDaS
            cv_image_rgb = cv2.cvtColor(cv_image_bgr, cv2.COLOR_BGR2RGB)
        except CvBridgeError as e:
            self.get_logger().error(f"Error CvBridge al recibir: {e}")
            return
        except Exception as e:
             self.get_logger().error(f"Error convirtiendo imagen recibida: {e}")
             return

        try:
            # --- Procesamiento MiDaS (igual que antes) ---
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
            # --- Fin Procesamiento MiDaS ---

            # --- Preparación para Publicar (igual que antes) ---
            # 1. Normalizar
            depth_map_normalized = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            # 2. Aplicar mapa de color
            depth_heatmap_bgr = cv2.applyColorMap(depth_map_normalized, COLORMAP_SELECCIONADO)
            # --- Fin Preparación ---

            # --- Publicación de la imagen procesada ---
            try:
                # Convertir la imagen OpenCV (numpy BGR) de vuelta a mensaje ROS Image
                # Usamos "bgr8" porque depth_heatmap_bgr es una imagen a color BGR de 8 bits
                img_msg_out = self.bridge.cv2_to_imgmsg(depth_heatmap_bgr, encoding="bgr8")
                
                # Copiar el timestamp del mensaje original al mensaje publicado
                # para mantener la referencia temporal (opcional pero buena práctica)
                img_msg_out.header.stamp = msg.header.stamp 
                # Publicar el mensaje en el topic de salida
                self.publisher_.publish(img_msg_out)
                
                # Log de confirmación (menos frecuente)
                self.get_logger().info("DEBUG: Mapa de profundidad procesado y publicado.", throttle_duration_sec=1.0)

            except CvBridgeError as e_pub:
                 self.get_logger().error(f"Error CvBridge al preparar para publicar: {e_pub}")
            except Exception as e_pub_gen:
                 self.get_logger().error(f"Error al publicar imagen: {e_pub_gen}")
        
        except Exception as e_proc:
            self.get_logger().error(f"Error durante procesamiento MiDaS: {e_proc}")
            # traceback.print_exc() # Descomentar para ver error detallado de MiDaS

        # --- Ya NO hay cv2.imshow ni cv2.waitKey ---


    def destroy_node(self):
        self.get_logger().info("Cerrando nodo publicador MiDaS...")
        # Ya NO hay cv2.destroyAllWindows
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    midas_publisher_node = None # Renombrada variable
    try:
        midas_publisher_node = MidasPublisherNode()
        if midas_publisher_node and rclpy.ok():
             midas_publisher_node.get_logger().info("Iniciando bucle rclpy.spin()...")
             rclpy.spin(midas_publisher_node)
        else:
             if not midas_publisher_node:
                  print("Error: No se pudo crear el nodo MidasPublisherNode.")
    except KeyboardInterrupt:
        print("Ctrl+C detectado, cerrando nodo...")
    except Exception as e:
        print(f"Error inesperado en main: {e}")
        traceback.print_exc()
    finally:
        print("Bloque finally en main...")
        if midas_publisher_node:
            try:
                midas_publisher_node.destroy_node()
            except Exception as destroy_e:
                 print(f"Error durante destroy_node: {destroy_e}")
        if rclpy.ok():
            print("Cerrando rclpy...")
            rclpy.shutdown()
        print("Programa finalizado.")

if __name__ == '__main__':
    main()