# tello_keyboard_controller_rgb_publisher.py
# Nodo ROS 2 para controlar el Tello con el teclado y publicar su vídeo en RGB.
"""
W, S, A, D: Movimiento adelante, atrás, izquierda, derecha.
Flecha Arriba, Flecha Abajo: Subir, bajar.
Q, E: Rotar izquierda, rotar derecha.
T: Despegar.
L: Aterrizar.
ESC: Detener el nodo y el listener de teclado (forma limpia de salir).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from djitellopy import Tello
import cv2
import time
import traceback
import threading
from pynput import keyboard # Para el control por teclado

# --- Configuración ---
TELLO_IP = '192.168.10.1' # IP por defecto del Tello en modo AP
ROS_TOPIC_OUTPUT = '/tello/camera/image_rgb'
PUBLISH_IMAGE_TIMER_PERIOD = 1.0 / 30.0  # ~30 FPS para publicar vídeo
SEND_RC_CONTROL_TIMER_PERIOD = 1.0 / 20.0 # Enviar comandos RC cada 50ms (20Hz)

# Parámetros de control
CONTROL_SPEED = 10  # Velocidad base para movimientos (0-100)
YAW_SPEED = 10      # Velocidad para rotación (0-100)

class TelloKeyboardControllerPublisher(Node):
    def __init__(self):
        super().__init__('tello_keyboard_controller_publisher')
        self.get_logger().info(f"Iniciando nodo controlador de Tello por teclado y publicador RGB...")
        self.get_logger().info(f"Conectando a Tello en IP: {TELLO_IP}")
        self.get_logger().info(f"Publicando imágenes RGB en: {ROS_TOPIC_OUTPUT}")
        self.get_logger().info(
            "Controles: W/S (Adelante/Atrás), A/D (Izquierda/Derecha), "
            "Flechas Arriba/Abajo (Subir/Bajar), Q/E (Rotar Izq/Der), "
            "T (Despegar), L (Aterrizar)"
        )

        self.bridge = CvBridge()
        self.tello = Tello(host=TELLO_IP)
        self.frame_reader = None
        self.tello_connected = False
        self.stream_on = False
        self.shutdown_requested = threading.Event()

        # Variables para send_rc_control [lr, fb, ud, yv]
        self.rc_forward_backward = 0
        self.rc_left_right = 0
        self.rc_up_down = 0
        self.rc_yaw = 0
        self.key_states = {} # Para rastrear qué teclas están presionadas

        # --- Conexión y configuración del Tello ---
        try:
            self.tello.connect()
            self.tello_connected = True
            self.get_logger().info(f"Conectado. Batería: {self.tello.get_battery()}%")
            if self.tello.get_battery() < 20:
                raise RuntimeError(f"Batería baja ({self.tello.get_battery()}%). Abortando.")

            self.get_logger().info("Configurando stream de vídeo...")
            self.tello.streamoff()
            time.sleep(0.5)
            self.tello.streamon()
            self.stream_on = True
            self.get_logger().info("Stream activado.")
            
            self.frame_reader = self.tello.get_frame_read()
            if self.frame_reader is None:
                raise RuntimeError("No se pudo obtener frame_reader del Tello.")
            self.get_logger().info("Obtenido frame_reader.")
            time.sleep(1.0)
            
        except Exception as e:
            self.get_logger().fatal(f"Error durante la inicialización del Tello: {e}")
            traceback.print_exc()
            self.cleanup_tello()
            raise SystemExit(f"Fallo en inicialización de Tello: {e}")

        # --- Publicador ROS para la imagen ---
        qos_profile_img = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.image_publisher_ = self.create_publisher(Image, ROS_TOPIC_OUTPUT, qos_profile_img)
        self.publish_image_timer = self.create_timer(PUBLISH_IMAGE_TIMER_PERIOD, self.publish_image_callback)
        self.get_logger().info("Publicador de imágenes y temporizador creados.")

        # --- Temporizador para enviar comandos RC ---
        self.rc_control_timer = self.create_timer(SEND_RC_CONTROL_TIMER_PERIOD, self.send_rc_control_callback)
        self.get_logger().info("Temporizador para send_rc_control creado.")

        # --- Iniciar Listener de Teclado ---
        self.keyboard_listener = keyboard.Listener(on_press=self.on_key_press, on_release=self.on_key_release)
        self.keyboard_listener.start()
        self.get_logger().info("Listener de teclado iniciado. El control está activo.")

    def update_rc_commands(self):
        """Actualiza los valores de RC basados en las teclas presionadas."""
        self.rc_forward_backward = 0
        self.rc_left_right = 0
        self.rc_up_down = 0
        self.rc_yaw = 0

        if self.key_states.get(keyboard.Key.up):
            self.rc_up_down = CONTROL_SPEED
        if self.key_states.get(keyboard.Key.down):
            self.rc_up_down = -CONTROL_SPEED
        
        if self.key_states.get(keyboard.KeyCode.from_char('w')):
            self.rc_forward_backward = CONTROL_SPEED
        if self.key_states.get(keyboard.KeyCode.from_char('s')):
            self.rc_forward_backward = -CONTROL_SPEED
        
        if self.key_states.get(keyboard.KeyCode.from_char('a')):
            self.rc_left_right = -CONTROL_SPEED
        if self.key_states.get(keyboard.KeyCode.from_char('d')):
            self.rc_left_right = CONTROL_SPEED
            
        if self.key_states.get(keyboard.KeyCode.from_char('q')):
            self.rc_yaw = -YAW_SPEED
        if self.key_states.get(keyboard.KeyCode.from_char('e')):
            self.rc_yaw = YAW_SPEED

    def on_key_press(self, key):
        if self.shutdown_requested.is_set() or not self.tello_connected:
            return

        # Comandos directos
        try:
            if key == keyboard.KeyCode.from_char('t'):
                self.get_logger().info("Tecla T: Despegando...")
                self.tello.takeoff()
            elif key == keyboard.KeyCode.from_char('l'):
                self.get_logger().info("Tecla L: Aterrizando...")
                self.tello.land()
                # Resetear movimientos al aterrizar
                self.key_states.clear()
                self.update_rc_commands() 
                self.tello.send_rc_control(0,0,0,0)


            # Teclas de movimiento (guardar estado)
            elif key in [keyboard.Key.up, keyboard.Key.down,
                         keyboard.KeyCode.from_char('w'), keyboard.KeyCode.from_char('s'),
                         keyboard.KeyCode.from_char('a'), keyboard.KeyCode.from_char('d'),
                         keyboard.KeyCode.from_char('q'), keyboard.KeyCode.from_char('e')]:
                self.key_states[key] = True
                self.update_rc_commands()
                
        except AttributeError: # Teclas especiales que no tienen .char
            if key in [keyboard.Key.up, keyboard.Key.down]:
                 self.key_states[key] = True
                 self.update_rc_commands()
            else:
                self.get_logger().debug(f"Tecla especial presionada: {key}")
        except Exception as e:
            self.get_logger().error(f"Error en on_key_press: {e}")

    def on_key_release(self, key):
        if self.shutdown_requested.is_set():
            return

        try:
            if key in self.key_states:
                del self.key_states[key]
                self.update_rc_commands()
        except Exception as e:
            self.get_logger().error(f"Error en on_key_release: {e}")
        
        # Detener listener si se presiona Esc (opcional, para salir limpiamente del listener)
        if key == keyboard.Key.esc:
            self.get_logger().info("Tecla ESC: Solicitando parada del nodo.")
            self.shutdown_requested.set() # Esto ayudará a salir del bucle de spin
            return False # Detener el listener de teclado


    def send_rc_control_callback(self):
        """ Envía periódicamente los comandos RC al Tello si está volando. """
        if not self.tello_connected or self.shutdown_requested.is_set() or not self.tello.is_flying:
            # Si no está volando pero hay comandos activos (ej. tras aterrizar y soltar tecla), enviar ceros una vez.
            if self.rc_left_right != 0 or self.rc_forward_backward != 0 or \
               self.rc_up_down != 0 or self.rc_yaw != 0:
                try:
                    # self.tello.send_rc_control(0, 0, 0, 0) # Comentado para evitar spam si no vuela
                    pass
                except Exception:
                    pass # Ignorar si hay error al enviar si no está conectado/volando
            return
        
        try:
            self.tello.send_rc_control(
                self.rc_left_right,
                self.rc_forward_backward,
                self.rc_up_down,
                self.rc_yaw
            )
        except Exception as e:
            self.get_logger().error(f"Error enviando send_rc_control: {e}", throttle_duration_sec=2.0)


    def publish_image_callback(self):
        """ Captura un frame del Tello, lo convierte a RGB y lo publica. """
        if self.frame_reader is None or not self.tello_connected or self.shutdown_requested.is_set():
            return
        if self.frame_reader.stopped:
            self.get_logger().warn("Frame reader del Tello detenido.", throttle_duration_sec=5.0)
            return

        frame_bgr = self.frame_reader.frame
        if frame_bgr is None:
            self.get_logger().warn("Frame del Tello es None.", throttle_duration_sec=5.0)
            return

        try:
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            
            ros_image_msg = self.bridge.cv2_to_imgmsg(frame_rgb, encoding="rgb8")
            ros_image_msg.header.stamp = self.get_clock().now().to_msg()
            ros_image_msg.header.frame_id = "tello_camera_rgb" # frame_id descriptivo
            self.image_publisher_.publish(ros_image_msg)
        except Exception as e:
            self.get_logger().error(f"Error publicando frame RGB: {e}", throttle_duration_sec=5.0)

    def cleanup_tello(self):
        self.get_logger().info("Iniciando limpieza de Tello...")
        self.shutdown_requested.set() # Señal para otros bucles/timers

        if hasattr(self, 'keyboard_listener') and self.keyboard_listener.is_alive():
            self.get_logger().info("Deteniendo listener de teclado...")
            self.keyboard_listener.stop() # No join() aquí para evitar bloqueo si está en on_press

        if hasattr(self, 'stream_on') and self.stream_on:
            self.get_logger().info("Deteniendo stream de vídeo del Tello...")
            try:
                self.tello.streamoff()
            except Exception as e:
                self.get_logger().error(f"Excepción en streamoff: {e}")
            self.stream_on = False
            
        if hasattr(self, 'tello_connected') and self.tello_connected:
            self.get_logger().info("Aterrizando el Tello si está volando...")
            try:
                if self.tello.is_flying: # Comprobar si realmente está volando
                    self.tello.land()
                    time.sleep(3) # Dar tiempo para aterrizar
            except Exception as e:
                self.get_logger().error(f"Error o no necesario aterrizar durante cleanup: {e}")

            self.get_logger().info("Desconectando del Tello...")
            try:
                self.tello.end()
            except Exception as e:
                self.get_logger().error(f"Excepción en tello.end(): {e}")
            self.tello_connected = False
        self.get_logger().info("Limpieza de Tello finalizada.")

    def destroy_node(self):
        self.get_logger().info("Destruyendo nodo...")
        if hasattr(self, 'publish_image_timer') and self.publish_image_timer:
            self.publish_image_timer.cancel()
        if hasattr(self, 'rc_control_timer') and self.rc_control_timer:
            self.rc_control_timer.cancel()
        
        self.cleanup_tello() # Llama a esto aquí para asegurar que Tello se maneje primero
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = TelloKeyboardControllerPublisher()
        
        # Bucle principal para mantener el nodo activo y procesar callbacks de ROS
        # y permitir que el nodo se cierre cuando shutdown_requested se active
        while rclpy.ok() and not node.shutdown_requested.is_set():
            rclpy.spin_once(node, timeout_sec=0.05) # Procesar eventos ROS sin bloquear demasiado

        if node.shutdown_requested.is_set():
            node.get_logger().info("Señal de shutdown recibida, saliendo del bucle principal.")

    except KeyboardInterrupt:
        print("\nCtrl+C detectado, iniciando cierre del nodo...")
        if node:
            node.shutdown_requested.set() 
    except SystemExit as e:
        print(f"Saliendo por SystemExit: {e}")
        if str(e).startswith("Fallo en inicialización de Tello"):
            exit_code = 1 # Indicar error en la salida
    except Exception as e:
        print(f"Error inesperado en main: {e}")
        traceback.print_exc()
        if node:
            node.shutdown_requested.set()
        exit_code = 1
    finally:
        print("Bloque finally en main: Limpiando recursos...")
        if node:
            node.destroy_node() # Asegura que destroy_node se llame
        
        if rclpy.ok(): # Solo si rclpy no ha sido cerrado ya por otra excepción
            print("Cerrando rclpy...")
            rclpy.shutdown()
        print(f"Programa finalizado con código de salida: {exit_code}")
        # Forzar salida si el listener de pynput no se detiene limpiamente
        import os
        os._exit(exit_code)


if __name__ == '__main__':
    main()