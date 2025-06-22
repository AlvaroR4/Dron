import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from djitellopy import Tello
import cv2
import time
import threading
from pynput import keyboard 

TELLO_IP = '192.168.10.1'
ROS_TOPIC_IMAGEN_RAW = '/tello/imagen_raw'
ROS_TOPIC_COMANDOS_VELOCIDAD = '/tello/comandos_velocidad'
TIMER_PERIODO_CAMARA = 1.0 / 30.0    # ~30 FPS para la cámara

class NodoTello(Node):
    def __init__(self):
        super().__init__('nodo_tello_tello')
        self.get_logger().info(f"Iniciando Nodo Camara Tello")

        self.bridge = CvBridge()
        self.tello = Tello(host=TELLO_IP)
        self.frame_reader = None 
        self.parada_emergencia_activa = False

        self.get_logger().info("Conectando al Tello ")
        self.tello.connect()
        self.tello.streamoff()
        time.sleep(0.5)
        self.tello.streamon()
        self.frame_reader = self.tello.get_frame_read() 
        time.sleep(1.0) 

        self.get_logger().info("Despegando Tello")
        self.tello.takeoff()
        time.sleep(2) 

        # Publicador para la imagen RAW
        qos_profile_img = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publicador_imagen_raw = self.create_publisher(Image, ROS_TOPIC_IMAGEN_RAW, qos_profile_img)
        self.get_logger().info(f"Publicando imagen RAW en: {ROS_TOPIC_IMAGEN_RAW}")

        # Suscriptor para los comandos de velocidad
        qos_profile_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.suscriptor_comandos_velocidad = self.create_subscription(
            Float32MultiArray,
            ROS_TOPIC_COMANDOS_VELOCIDAD,
            self.callback_comandos_velocidad,
            qos_profile_cmd
        )
        self.get_logger().info(f"Suscrito a comandos de velocidad en: {ROS_TOPIC_COMANDOS_VELOCIDAD}")

        # Timer para captura y publicación de imagen
        self.timer_camara = self.create_timer(TIMER_PERIODO_CAMARA, self.timer_callback_camara)
        self.get_logger().info(f"Timer de cámara iniciado ({1.0/TIMER_PERIODO_CAMARA:.1f} FPS).")


    def callback_comandos_velocidad(self, msg):
        lr, fb, ud, yv = int(msg.data[0]), int(msg.data[1]), int(msg.data[2]), int(msg.data[3])
        self.tello.send_rc_control(lr, fb, ud, yv)

    def timer_callback_camara(self):
        if self.parada_emergencia_activa:
            return

        frame_bgr = self.frame_reader.frame # Obtenemos el frame

        if frame_bgr is None:
            self.get_logger().warn("Frame de la cámara es None.", throttle_duration_sec=2.0)
            return

        # Publicar el frame
        ros_image_msg = self.bridge.cv2_to_imgmsg(frame_bgr, encoding="bgr8")
        ros_image_msg.header.stamp = self.get_clock().now().to_msg()
        ros_image_msg.header.frame_id = "tello_camera_link_raw"
        self.publicador_imagen_raw.publish(ros_image_msg)

    def cleanup_recursos(self):

        self.get_logger().info("Iniciando limpieza de recursos del Tello...")

        # Detener RC control
        self.tello.send_rc_control(0, 0, 0, 0)
        time.sleep(0.1)

        # Si no hubo una emergencia activa, intentar aterrizar.
        if not self.parada_emergencia_activa:
            self.get_logger().info("Aterrizando el Tello...")
            self.tello.land()
            time.sleep(5)

        self.get_logger().info("Deteniendo stream de vídeo del Tello...")
        self.tello.streamoff()

        self.get_logger().info("Desconectando del Tello...")
        self.tello.end()
        self.get_logger().info("Limpieza de Tello finalizada.")

    def destroy_node(self):
        if self.timer_camara:
            self.timer_camara.cancel()

        self.cleanup_recursos()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    nodo_tello = NodoTello() 
    try:
        rclpy.spin(nodo_tello)
    except KeyboardInterrupt:
        nodo_tello.parada_emergencia_activa = True
        nodo_tello.tello.land()
        nodo_tello.get_logger().info("Ctrl+C detectado. Iniciando cierre del nodo.")
    finally:
        nodo_tello.destroy_node()
        rclpy.shutdown()
        print("Programa NodoTello finalizado.")

if __name__ == '__main__':
    main()
