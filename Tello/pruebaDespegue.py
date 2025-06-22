# tello_takeoff_hover_land_rgb.py
# Nodo ROS 2 que publica el vídeo del Tello en formato RGB y ejecuta una secuencia
# simple de despegue -> hover (espera) -> aterrizaje en un hilo separado.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from djitellopy import Tello
import cv2
import time
import traceback
import threading

TELLO_IP = '192.168.10.1'
ROS_TOPIC_OUTPUT = '/tello/camara'
PUBLISH_TIMER_PERIOD = 1.0 / 30.0 # ~30 FPS para publicar vídeo
HOVER_DURATION_SEC = 20 # Tiempo en segundos que el dron esperará en el aire

class TelloFlightPublisher(Node):
    def __init__(self):
        super().__init__('tello_flight_publisher')
        self.get_logger().info(f"Iniciando nodo publicador y de vuelo simple del Tello...")
        self.get_logger().info(f"Intentando conectar al Tello en IP: {TELLO_IP}")
        self.get_logger().info(f"Publicando imágenes RGB en: {ROS_TOPIC_OUTPUT}")
        self.get_logger().info(f"Secuencia: Takeoff -> Hover {HOVER_DURATION_SEC}s -> Land")

        self.bridge = CvBridge()
        self.tello = Tello(host=TELLO_IP)
        self.frame_reader = None
        self.tello_connected = False
        self.stream_on = False
        self.shutdown_requested = threading.Event() # Evento para señalar parada desde el hilo de vuelo

        # --- Conexión y configuración del Tello ---
        try:
            self.tello.connect()
            self.tello_connected = True
            self.get_logger().info(f"Conectado. Batería: {self.tello.get_battery()}%")
            if self.tello.get_battery() < 15:
                 raise RuntimeError(f"Batería baja ({self.tello.get_battery()}%). Abortando vuelo.")

            self.get_logger().info("Configurando stream de vídeo...")
            self.tello.streamoff(); time.sleep(0.5); self.tello.streamon()
            self.stream_on = True
            self.get_logger().info("Stream activado.")
            
            self.frame_reader = self.tello.get_frame_read()
            if self.frame_reader is None: raise RuntimeError("frame_reader es None.")
            self.get_logger().info("Obtenido frame_reader.")
            time.sleep(1.0) 
            
        except Exception as e:
            self.get_logger().fatal(f"Error durante la inicialización del Tello: {e}")
            traceback.print_exc()
            self.cleanup_tello() # Intentar limpiar
            raise SystemExit(f"Fallo en inicialización de Tello: {e}")

        # --- Crear Publicador ROS ---
        qos_profile_publisher = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.image_publisher_ = self.create_publisher(Image, ROS_TOPIC_OUTPUT, qos_profile_publisher)
        self.get_logger().info("Publicador ROS creado.")

        # --- Crear Temporizador ROS para publicar frames ---
        self.publish_timer = self.create_timer(PUBLISH_TIMER_PERIOD, self.publish_timer_callback)
        self.get_logger().info(f"Temporizador de publicación creado con periodo: {PUBLISH_TIMER_PERIOD:.3f}s")

        # --- Iniciar Hilo de Vuelo ---
        self.flight_thread = threading.Thread(target=self.flight_sequence_thread, daemon=True)
        self.flight_thread.start()
        self.get_logger().info("Hilo de secuencia de vuelo iniciado.")


    def publish_timer_callback(self):
        if self.frame_reader is None or not self.tello_connected or self.shutdown_requested.is_set(): return
        if self.frame_reader.stopped: return

        frame_bgr = self.frame_reader.frame 
        if frame_bgr is None: return

        try:
            # Convertir el frame de BGR a RGB
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            
            # Crear el mensaje ROS Image con el frame RGB
            ros_image_msg = self.bridge.cv2_to_imgmsg(frame_rgb, encoding="rgb8")
            ros_image_msg.header.stamp = self.get_clock().now().to_msg()
            ros_image_msg.header.frame_id = "tello_camera"
            self.image_publisher_.publish(ros_image_msg)
        except Exception as e:
            self.get_logger().error(f"Error publicando frame RGB: {e}", throttle_duration_sec=5.0)

    def flight_sequence_thread(self):
        """ Ejecuta la secuencia de vuelo en un hilo separado """
        try:
            time.sleep(2.0) 
            
            if not self.tello_connected or self.shutdown_requested.is_set():
                self.get_logger().warn("Hilo de vuelo: Tello no conectado o parada solicitada antes de despegar.")
                return

            self.get_logger().info("Hilo de vuelo: Enviando comando takeoff...")
            self.tello.takeoff()
            self.get_logger().info("Hilo de vuelo: Takeoff completado (esperando estabilización).")
            time.sleep(5.0) 

            if self.shutdown_requested.is_set(): return

            self.get_logger().info(f"Hilo de vuelo: Iniciando hover de {HOVER_DURATION_SEC} segundos...")
            start_hover_time = time.time()
            while time.time() - start_hover_time < HOVER_DURATION_SEC:
                if self.shutdown_requested.is_set():
                    self.get_logger().info("Hilo de vuelo: Hover interrumpido.")
                    break 
                time.sleep(0.5)
            
            if self.shutdown_requested.is_set(): 
                 self.get_logger().warn("Hilo de vuelo: Parada solicitada antes de aterrizar.")
                 if time.time() - start_hover_time >= HOVER_DURATION_SEC: 
                      pass 
                 else:
                      return 

            self.get_logger().info(f"Hilo de vuelo: Hover completado ({HOVER_DURATION_SEC}s). Enviando comando land...")
            self.tello.land()
            self.get_logger().info("Hilo de vuelo: Land enviado (esperando aterrizaje).")
            time.sleep(5.0) 

        except Exception as e:
            self.get_logger().error(f"Error en hilo de vuelo: {e}")
            traceback.print_exc()
            try:
                 if self.tello.is_flying:
                      self.get_logger().warn("Intentando aterrizaje de emergencia desde hilo de vuelo...")
                      self.tello.land()
            except Exception as land_err:
                 self.get_logger().error(f"Error en aterrizaje de emergencia: {land_err}")

        finally:
            self.get_logger().info("Hilo de vuelo: Secuencia finalizada. Solicitando cierre del nodo...")
            self.shutdown_requested.set()


    def cleanup_tello(self):
        """ Limpieza específica del Tello """
        self.get_logger().info("Iniciando limpieza de Tello...")
        self.shutdown_requested.set() 
        
        if hasattr(self, 'stream_on') and self.stream_on:
            self.get_logger().info("Deteniendo stream...")
            try: self.tello.streamoff()
            except Exception as e: self.get_logger().error(f"Excepción en streamoff: {e}", throttle_duration_sec=5.0)
            self.stream_on = False
            
        if hasattr(self, 'tello_connected') and self.tello_connected:
            try:
                 if self.tello.is_flying:
                      self.get_logger().warn("Cleanup: El dron todavía está volando. Intentando aterrizar...")
                      self.tello.land()
                      time.sleep(5) 
            except Exception as land_err:
                 self.get_logger().error(f"Error en aterrizaje durante cleanup: {land_err}")

            self.get_logger().info("Desconectando del Tello...")
            try: self.tello.end()
            except Exception as e: self.get_logger().error(f"Excepción en end: {e}", throttle_duration_sec=5.0)
            self.tello_connected = False
        self.get_logger().info("Limpieza de Tello finalizada.")

    def destroy_node(self):
        self.get_logger().info("Cerrando nodo de vuelo Tello...")
        if hasattr(self, 'publish_timer') and self.publish_timer: self.publish_timer.cancel()
        self.cleanup_tello()
        if hasattr(self, 'flight_thread') and self.flight_thread.is_alive():
             self.get_logger().info("Esperando al hilo de vuelo...")
             self.flight_thread.join(timeout=2.0) 
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TelloFlightPublisher()
        if rclpy.ok():
             while rclpy.ok() and not node.shutdown_requested.is_set():
                  rclpy.spin_once(node, timeout_sec=0.1) 
             
             if node.shutdown_requested.is_set():
                  print("Señal de shutdown recibida del hilo de vuelo.")
             else: 
                  print("Saliendo del bucle principal...")

        else: 
             if not node: print("Error: No se pudo crear el nodo.")
             
    except KeyboardInterrupt:
        print("Ctrl+C detectado, iniciando cierre...")
        if node: node.shutdown_requested.set() 
    except SystemExit as e:
        print(f"Saliendo: {e}")
    except Exception as e:
        print(f"Error inesperado en main: {e}")
        traceback.print_exc()
    finally:
        print("Bloque finally en main...")
        if node:
            try:
                 if not rclpy.ok() or node.shutdown_requested.is_set(): 
                      node.destroy_node()
            except Exception as destroy_e:
                 print(f"Error durante destroy_node explícito: {destroy_e}")

        if rclpy.ok():
            print("Cerrando rclpy...")
            rclpy.shutdown()
        print("Programa finalizado.")


if __name__ == '__main__':
    main()