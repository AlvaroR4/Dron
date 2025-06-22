import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from djitellopy import Tello
import cv2
import numpy as np 
import time
import threading 
from pynput import keyboard 
import traceback 

TELLO_IP = '192.168.10.1'
ROS_TOPIC_IMAGEN_RAW = '/tello/imagen_controlTeclado' 
TIMER_PERIODO_CAMARA = 1.0 / 30.0 
FRAME_WIDTH_PROC = 640
FRAME_HEIGHT_PROC = 480

class NodoTello(Node):
    def __init__(self):
        super().__init__('nodo_tello_tello')
        self.get_logger().info(f"Iniciando Nodo Control por Teclado para Tello")

        self.bridge = CvBridge()
        self.tello = Tello(host=TELLO_IP)
        self.frame_reader = None
        self.presionadoS = False

        self.get_logger().info("Conectando al Tello ")
        self.tello.connect()
        if not self.tello.is_connected:
            self.get_logger().fatal("No se pudo conectar con el Tello. Abortando.")
            raise RuntimeError("Fallo en la conexión con el Tello.")

        self.tello.streamoff()
        time.sleep(0.5)
        self.tello.streamon()
        self.frame_reader = self.tello.get_frame_read()
        if self.frame_reader is None:
            self.get_logger().fatal("No se pudo obtener frame_reader del Tello. Abortando.")
            raise RuntimeError("Fallo al iniciar el stream de vídeo del Tello.")
        time.sleep(1.0)

        self.get_logger().info("Despegando Tello")
        self.tello.takeoff()
        time.sleep(2)

        # Publicador para la imagen 
        qos_profile_img = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publicador_imagen_raw = self.create_publisher(Image, ROS_TOPIC_IMAGEN_RAW, qos_profile_img)
        self.get_logger().info(f"Publicando imagen RAW en: {ROS_TOPIC_IMAGEN_RAW}")


        # Timer para captura y publicación de imagen
        self.timer_camara = self.create_timer(TIMER_PERIODO_CAMARA, self.timer_callback_camara)
        self.get_logger().info(f"Timer de cámara iniciado ({1.0/TIMER_PERIODO_CAMARA:.1f} FPS).")

        #Hilo para el control manual por teclado
        self.hilo_control_manual = threading.Thread(target=self.control_manual_tello, daemon=True)
        self.hilo_control_manual.start()
        self.get_logger().info("Hilo de control manual por teclado iniciado. Pulsa 's' para reiniciar velocidades")


        self.listener_teclado = keyboard.Listener(on_press=self.on_press_teclado)
        self.listener_teclado.start()


    def on_press_teclado(self, key):
        try:
            if key.char == 's':
                self.presionadoS = True
        except AttributeError:
            pass 

    def control_manual_tello(self):
        self.get_logger().info("Introduce velocidades (lr, fb, ud, yv) separadas por comas. Pulsa 's' para detener y volver a pedir.")
        self.get_logger().info("Valores de -100 a 100. Ejemplo: 10,20,-20,0")

        current_lr, current_fb, current_ud, current_yv = 0, 0, 0, 0 

        while rclpy.ok():
            try:
                if self.presionadoS:
                    entrada = input("Introduce velocidades (lr,fb,ud,yv) o 's' para parar: ").strip()

                    if entrada.lower() == 's':
                        self.get_logger().info("Parando movimiento (velocidades a 0).")
                        current_lr, current_fb, current_ud, current_yv = 0, 0, 0, 0
                        continue 

                    partes = entrada.split(',')
                    if len(partes) == 4:
                        try:
                            lr = int(partes[0])
                            fb = int(partes[1])
                            ud = int(partes[2])
                            yv = int(partes[3])

                            lr = np.clip(lr, -100, 100)
                            fb = np.clip(fb, -100, 100)
                            ud = np.clip(ud, -100, 100)
                            yv = np.clip(yv, -100, 100)

                            current_lr, current_fb, current_ud, current_yv = lr, fb, ud, yv
                            self.presionadoS = False
                            self.get_logger().info(f"Enviando continuamente: lr={lr}, fb={fb}, ud={ud}, yv={yv}. Pulsa 's' para detener.")

                        except ValueError:
                            self.get_logger().warn("Formato de velocidades incorrecto. Usa números enteros. Ejemplo: 10,20,-20,0")
                    else:
                        self.get_logger().warn("Número incorrecto de valores. Se esperan 4: lr, fb, ud, yv.")
                else: 
                    self.enviar_comandos_tello_directo(current_lr, current_fb, current_ud, current_yv)
                    time.sleep(0.1) 

            except Exception as e:
                self.get_logger().error(traceback.format_exc())
                break
        self.get_logger().info("Hilo de control manual finalizado.")
        self.enviar_comandos_tello_directo(0,0,0,0)


    def enviar_comandos_tello_directo(self, lr, fb, ud, yv):
        self.tello.send_rc_control(lr, fb, ud, yv)
        # self.get_logger().debug(f"Comando RC directo: {lr},{fb},{ud},{yv}")


    def timer_callback_camara(self):
        frame_bgr = self.frame_reader.frame 

        if frame_bgr is None:
            self.get_logger().warn("Frame de la cámara es None.", throttle_duration_sec=2.0)
            return

        img_proc_bgr = cv2.resize(frame_bgr, (FRAME_WIDTH_PROC, FRAME_HEIGHT_PROC))
        img_final_rgb = cv2.cvtColor(img_proc_bgr, cv2.COLOR_BGR2RGB)


        ros_image_msg = self.bridge.cv2_to_imgmsg(img_final_rgb, encoding="rgb8")
        ros_image_msg.header.stamp = self.get_clock().now().to_msg()
        ros_image_msg.header.frame_id = "tello_camera"
        self.publicador_imagen_raw.publish(ros_image_msg)

    def cleanup_recursos(self):
        self.tello.send_rc_control(0, 0, 0, 0)
        time.sleep(0.1)

        self.get_logger().info("Aterrizando el Tello...")
        self.tello.land()
        time.sleep(5)

        self.tello.streamoff()
        self.tello.end()

    def destroy_node(self):
        self.get_logger().info("Destruyendo NodoTello...")
        if hasattr(self, 'timer_camara') and self.timer_camara:
            self.timer_camara.cancel()

        if hasattr(self, 'listener_teclado') and self.listener_teclado.is_alive():
            self.listener_teclado.stop()

        self.cleanup_recursos() 
        super().destroy_node() 

def main(args=None):
    rclpy.init(args=args)
    nodo_tello = None
    try:
        nodo_tello = NodoTello()
        rclpy.spin(nodo_tello) 

    except KeyboardInterrupt:
        if nodo_tello:
            nodo_tello.get_logger().info("Ctrl+C detectado. Iniciando cierre del nodo.")
            nodo_tello.tello.land()
    except RuntimeError as e_runtime:
        if nodo_tello:
            nodo_tello.get_logger().fatal(f"Error crítico durante la inicialización: {e_runtime}")
        else:
            print(f"Error crítico durante la inicialización del nodo: {e_runtime}")
    except Exception as e_main:
        if nodo_tello:
            nodo_tello.get_logger().fatal(f"Error inesperado en main de NodoTello: {e_main}")
            traceback.print_exc() # Descomentar para ver el stack trace
        else:
            print(f"Error inesperado en main antes de inicializar NodoTello: {e_main}")
            traceback.print_exc()
    finally:
        if nodo_tello:
            nodo_tello.destroy_node() # Esto llamará a cleanup_recursos
        if rclpy.ok():
            rclpy.shutdown()
        print("Programa NodoTello finalizado.")

if __name__ == '__main__':
    main()