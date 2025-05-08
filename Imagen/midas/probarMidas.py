"""
probarMidasRos.py
Se suscribe a un topic de imagen ROS 2, aplica MiDaS para estimar profundidad
y muestra la imagen original y el mapa de profundidad en tiempo real usando Matplotlib.

Dependencias: 
pip install rclpy opencv-python-headless cv_bridge torch torchvision matplotlib requests numpy
pip install timm
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import torch
import numpy as np
import matplotlib.pyplot as plt

# --- Configuración ---
CAMARA_TOPIC = '/world/puertas3/model/x500_mono_cam_0/link/camera_link/sensor/imager/image'
# Modelo MiDaS a usar ("MiDaS_small", "MiDaS", "DPT_Large", "DPT_Hybrid")
# MiDaS_small es más rápido, DPT_Large más preciso
MODEL_TYPE = "MiDaS_small"

class MidasVisualizerNode(Node):
    def __init__(self):
        super().__init__('midas_visualizer_node')
        self.get_logger().info(f"Iniciando Nodo Visualizador MiDaS para topic: {CAMARA_TOPIC}")

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
            self.get_logger().fatal(f"Error cargando modelo MiDaS: {e}. Asegúrate de tener conexión a internet.")
            rclpy.shutdown() # Detener el nodo si no se puede cargar el modelo
            return

        # Configurar Matplotlib para visualización interactiva
        plt.ion() # Activar modo interactivo
        self.fig, self.axs = plt.subplots(1, 2, figsize=(12, 5)) # 1 fila, 2 columnas
        self.fig.canvas.manager.set_window_title('Visor MiDaS en Tiempo Real')
        self.im_orig = None # Placeholder para la imagen original
        self.im_depth = None # Placeholder para la imagen de profundidad
        self.colorbar = None # Placeholder para la barra de color

        self.axs[0].set_title("Cámara Original (RGB)")
        self.axs[0].axis('off')
        self.axs[1].set_title("Mapa Profundidad Estimada (MiDaS)")
        self.axs[1].axis('off')
        plt.show(block=False) # Mostrar ventana sin bloquear ejecución

    def listener_callback(self, msg):
        self.get_logger().info(f'Recibiendo frame: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}', throttle_duration_sec=1.0) # Log reducido
        try:
            # Convertir mensaje ROS Image a imagen OpenCV (BGR)
            cv_image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Convertir a RGB para MiDaS y Matplotlib
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

                # Post-procesar (interpolar a tamaño original)
                prediction = torch.nn.functional.interpolate(
                    prediction.unsqueeze(1),
                    size=cv_image_rgb.shape[:2],
                    mode="bicubic",
                    align_corners=False,
                ).squeeze()

            # Mover a CPU y convertir a NumPy
            depth_map = prediction.cpu().numpy()

            # Actualizar la visualización de Matplotlib
            if self.im_orig is None: # Si es el primer frame, crear los plots
                self.im_orig = self.axs[0].imshow(cv_image_rgb)
                self.im_depth = self.axs[1].imshow(depth_map, cmap="inferno")
                self.colorbar = self.fig.colorbar(self.im_depth, ax=self.axs[1], label='Profundidad Relativa Inversa')
            else: # Si ya existen, solo actualizar datos
                self.im_orig.set_data(cv_image_rgb)
                self.im_depth.set_data(depth_map)
                # Ajustar límites de la barra de color dinámicamente
                self.im_depth.set_clim(vmin=np.min(depth_map), vmax=np.max(depth_map))
                # Re-dibujar solo si es necesario ( draw_idle es más eficiente)
                self.fig.canvas.draw_idle()
                self.fig.canvas.flush_events() # Procesar eventos de la GUI

        except Exception as e:
            self.get_logger().error(f"Error durante procesamiento MiDaS o visualización: {e}")


    def destroy_node(self):
        self.get_logger().info("Cerrando nodo visualizador MiDaS...")
        plt.ioff() # Desactivar modo interactivo
        plt.close(self.fig) # Cerrar figura de matplotlib
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    midas_visualizer_node = None
    try:
        midas_visualizer_node = MidasVisualizerNode()
        rclpy.spin(midas_visualizer_node)
    except KeyboardInterrupt:
        print("Ctrl+C detectado, cerrando nodo...")
    except Exception as e:
        if midas_visualizer_node:
            midas_visualizer_node.get_logger().fatal(f"Error inesperado: {e}")
        else:
            print(f"Error durante inicialización: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        if midas_visualizer_node:
            midas_visualizer_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Programa finalizado.")

if __name__ == '__main__':
    main()