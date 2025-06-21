# Nodo ROS 2 que se conecta al Tello, obtiene su stream de vídeo
# y publica los frames en un topic.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from djitellopy import Tello
import cv2
import time

TELLO_IP = '192.168.10.1'
ROS_TOPIC_OUTPUT = '/tello/imagen'
TIMER_PERIOD = 1.0 / 30.0 # Periodo del temporizador en segundos (aprox. 30 FPS)

FRAME_WIDTH_PROC = 640
FRAME_HEIGHT_PROC = 480

class TelloImagePublisher(Node):
    def __init__(self):
        super().__init__('tello_image_publisher')
        self.get_logger().info("Iniciando nodo publicador de imágenes del Tello.")
        self.get_logger().info(f"Intentando conectar al Tello en IP: {TELLO_IP}")
        self.get_logger().info(f"Publicando imágenes en: {ROS_TOPIC_OUTPUT}")

        self.bridge = CvBridge()
        self.tello = Tello(host=TELLO_IP)
        self.frame_reader = None
        
        # Conexión y configuración del Tello
        self.tello.connect()
        self.get_logger().info(f"Conectado al Tello. Batería: {self.tello.get_battery()}%")

        self.get_logger().info("Configurando stream de vídeo...")
        self.tello.streamoff()
        time.sleep(0.5)
        self.tello.streamon()
        self.get_logger().info("Stream activado.")
        
        self.frame_reader = self.tello.get_frame_read()
        self.get_logger().info("Obtenido frame_reader. Esperando estabilización...")
        time.sleep(1.5) # Pausa para estabilizar
            
        # Crear Publicador ROS
        qos_profile_publisher = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.image_publisher_ = self.create_publisher(Image, ROS_TOPIC_OUTPUT, qos_profile_publisher)
        self.get_logger().info("Publicador ROS creado.")

        # Crear Temporizador ROS para publicar frames
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)
        self.get_logger().info(f"Temporizador creado con periodo: {TIMER_PERIOD:.3f}s")


    def timer_callback(self):
        """ Se ejecuta periódicamente para obtener y publicar un frame """
        frame = self.frame_reader.frame
        
        if frame is None:
            self.get_logger().warn("No se han recibido frames válidos.", throttle_duration_sec=5.0)
            return # Salir del callback si no hay frame

        img_proc_bgr = cv2.resize(frame, (FRAME_WIDTH_PROC, FRAME_HEIGHT_PROC))
        img_proc_rgb = cv2.cvtColor(img_proc_bgr, cv2.COLOR_BGR2RGB) #
        
        # Publicamos en RGB
        ros_image_msg = self.bridge.cv2_to_imgmsg(img_proc_rgb, encoding="rgb8")
        
        # Añadir timestamp y frame_id al mensaje ROS
        ros_image_msg.header.stamp = self.get_clock().now().to_msg()
        ros_image_msg.header.frame_id = "tello_camera"
        
        # Publicar el mensaje
        self.image_publisher_.publish(ros_image_msg)
        self.get_logger().debug("Frame publicado en ROS.", throttle_duration_sec=1.0)


    def destroy_node(self):
        """ Se llama automáticamente al cerrar el nodo """
        self.get_logger().info("Cerrando nodo publicador Tello...")
        if self.timer:
            self.timer.cancel() # Detener el temporizador
        
        self.get_logger().info("Deteniendo stream del Tello...")
        self.tello.streamoff()
        self.get_logger().info("Desconectando del Tello...")
        self.tello.end()
        self.get_logger().info("Limpieza de Tello finalizada.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    tello_image_publisher_node = TelloImagePublisher()
    try:
        rclpy.spin(tello_image_publisher_node)
    except KeyboardInterrupt:
        print("Ctrl+C detectado, cerrando nodo...")
    finally:
        tello_image_publisher_node.destroy_node()
        rclpy.shutdown()
        print("Programa finalizado.")

if __name__ == '__main__':
    main()
