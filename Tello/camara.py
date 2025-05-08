# Nodo ROS 2 que se conecta al Tello, obtiene su stream de vídeo
# y publica los frames en un topic sensor_msgs/msg/Image.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from djitellopy import Tello
import cv2
import time
import traceback
import threading # Para asegurar que el frame_reader se maneja bien

# --- Configuración ---
TELLO_IP = '192.168.10.1' # IP por defecto del Tello en modo AP
ROS_TOPIC_OUTPUT = '/tello/image_raw' # Topic donde se publicarán las imágenes
TIMER_PERIOD = 1.0 / 30.0 # Periodo del temporizador en segundos (aprox. 30 FPS)

class TelloImagePublisher(Node):
    def __init__(self):
        super().__init__('tello_image_publisher')
        self.get_logger().info(f"Iniciando nodo publicador de imágenes del Tello...")
        self.get_logger().info(f"Intentando conectar al Tello en IP: {TELLO_IP}")
        self.get_logger().info(f"Publicando imágenes en: {ROS_TOPIC_OUTPUT}")

        self.bridge = CvBridge()
        self.tello = Tello(host=TELLO_IP)
        self.frame_reader = None
        self.tello_connected = False
        self.stream_on = False

        # --- Conexión y configuración del Tello ---
        try:
            self.tello.connect()
            self.tello_connected = True
            self.get_logger().info(f"Conectado al Tello. Batería: {self.tello.get_battery()}%")

            self.get_logger().info("Configurando stream de vídeo...")
            self.tello.streamoff()
            time.sleep(0.5)
            self.tello.streamon()
            self.stream_on = True
            self.get_logger().info("Stream activado.")
            
            self.frame_reader = self.tello.get_frame_read()
            if self.frame_reader is None:
                 raise RuntimeError("No se pudo obtener frame_reader del Tello.")
            self.get_logger().info("Obtenido frame_reader. Esperando estabilización...")
            time.sleep(1.5) # Pausa para estabilizar
            
        except Exception as e:
            self.get_logger().fatal(f"Error durante la inicialización del Tello: {e}")
            traceback.print_exc()
            # Limpiar si falla la inicialización
            if self.stream_on: self.tello.streamoff()
            if self.tello_connected: self.tello.end()
            rclpy.shutdown() # Detener ROS si no podemos continuar
            raise SystemExit("Fallo en inicialización de Tello.")

        # --- Crear Publicador ROS ---
        qos_profile_publisher = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1 # Para vídeo crudo, solo el último frame suele importar
        )
        self.image_publisher_ = self.create_publisher(Image, ROS_TOPIC_OUTPUT, qos_profile_publisher)
        self.get_logger().info("Publicador ROS creado.")

        # --- Crear Temporizador ROS para publicar frames ---
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)
        self.get_logger().info(f"Temporizador creado con periodo: {TIMER_PERIOD:.3f}s")
        self.last_frame_time = time.time()


    def timer_callback(self):
        """ Se ejecuta periódicamente para obtener y publicar un frame """
        if self.frame_reader is None or not self.tello_connected:
            self.get_logger().warn("Frame reader o conexión no disponibles.", throttle_duration_sec=5.0)
            return
            
        if self.frame_reader.stopped:
            self.get_logger().error("El lector de frames del Tello se detuvo inesperadamente.")
            self.cleanup_tello() # Intentar limpiar
            rclpy.shutdown() # Detener el nodo
            return

        frame = self.frame_reader.frame
        current_time = time.time()
        
        if frame is None:
            # Imprimir solo si ha pasado un tiempo sin frames
            if current_time - self.last_frame_time > 2.0:
                 self.get_logger().warn("No se han recibido frames válidos recientemente.", throttle_duration_sec=5.0)
            return # Salir del callback si no hay frame

        self.last_frame_time = current_time # Actualizar tiempo del último frame válido

        try:
            # Convertir frame de OpenCV (BGR) a mensaje ROS Image
            # El frame del Tello viene en BGR
            ros_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            
            # Añadir timestamp al mensaje ROS
            ros_image_msg.header.stamp = self.get_clock().now().to_msg()
            ros_image_msg.header.frame_id = "tello_camera" # Opcional: nombre del frame
            
            # Publicar el mensaje
            self.image_publisher_.publish(ros_image_msg)
            self.get_logger().debug("Frame publicado en ROS.", throttle_duration_sec=1.0) # Debug para no saturar

        except CvBridgeError as e:
            self.get_logger().error(f"Error CvBridge al publicar: {e}")
        except Exception as e:
            self.get_logger().error(f"Error inesperado en timer_callback: {e}")
            # traceback.print_exc() # Descomentar para errores detallados

    def cleanup_tello(self):
        """ Método para intentar apagar el Tello de forma segura """
        self.get_logger().info("Iniciando limpieza de Tello...")
        if self.stream_on:
            self.get_logger().info("Deteniendo stream...")
            try: self.tello.streamoff()
            except Exception as e: self.get_logger().error(f"Excepción en streamoff: {e}", throttle_duration_sec=5.0)
            self.stream_on = False # Marcar como apagado incluso si falla
        if self.tello_connected:
            self.get_logger().info("Desconectando del Tello...")
            try: self.tello.end()
            except Exception as e: self.get_logger().error(f"Excepción en end: {e}", throttle_duration_sec=5.0)
            self.tello_connected = False
        self.get_logger().info("Limpieza de Tello finalizada.")


    def destroy_node(self):
        """ Se llama automáticamente al cerrar el nodo """
        self.get_logger().info("Cerrando nodo publicador Tello...")
        if self.timer:
            self.timer.cancel() # Detener el temporizador
        self.cleanup_tello() # Llamar a la limpieza específica del Tello
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    tello_image_publisher_node = None
    try:
        tello_image_publisher_node = TelloImagePublisher()
        if rclpy.ok(): # Comprobar si el nodo se inicializó bien antes de girar
            rclpy.spin(tello_image_publisher_node)
    except KeyboardInterrupt:
        print("Ctrl+C detectado, cerrando nodo...")
    except SystemExit as e:
        print(f"Saliendo debido a error de inicialización: {e}")
    except Exception as e:
        print(f"Error inesperado en main: {e}")
        traceback.print_exc()
    finally:
        # La limpieza se maneja en destroy_node, pero aseguramos el shutdown
        if tello_image_publisher_node:
             # destroy_node se llama automáticamente al salir de spin o por excepción si está bien gestionado
             # pero podemos llamarlo explícitamente si queremos asegurarnos fuera del spin
             try:
                 if rclpy.ok(): # Solo si no se llamó ya desde el shutdown
                     tello_image_publisher_node.destroy_node()
             except Exception as destroy_e:
                 print(f"Error adicional durante destroy_node explícito: {destroy_e}")
        
        if rclpy.ok():
            print("Cerrando rclpy...")
            rclpy.shutdown()
        print("Programa finalizado.")

if __name__ == '__main__':
    main()