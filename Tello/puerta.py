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
ROS_IMAGE_TOPIC_OUTPUT = '/tello/image_processed_rgb'
MAIN_TIMER_PERIOD = 1.0 / 20.0

FRAME_WIDTH_PROC = 640
FRAME_HEIGHT_PROC = 480
MIN_CONTOUR_AREA_TELLO = 2500

COLOR_LOWER_1 = np.array([0, 130, 90])
COLOR_UPPER_1 = np.array([10, 255, 255])
COLOR_LOWER_2 = np.array([160, 130, 90])
COLOR_UPPER_2 = np.array([179, 255, 255])

VELOCIDAD_AVANCE = 25
VELOCIDAD_CORRECCION = 30
VELOCIDAD_YAW = 0
MARGEN_ERROR_X_PIXELS = 30
MARGEN_ERROR_Y_PIXELS = 30
TIEMPO_AVANCE_EXTRA_SEG = 2.0
CONTADOR_PERDIDO_MAX = 15

ESTADO_INICIO = 0
ESTADO_DESPEGANDO = 10
ESTADO_BUSCANDO_ALINEANDO = 1
ESTADO_AVANZANDO = 2
ESTADO_POST_PUERTA = 3
ESTADO_ATERRIZANDO = 4
ESTADO_FINALIZADO = 5

estado_nombres = {
    ESTADO_INICIO: "INICIO",
    ESTADO_DESPEGANDO: "DESPEGANDO",
    ESTADO_BUSCANDO_ALINEANDO: "BUSCANDO/ALINEANDO PUERTA",
    ESTADO_AVANZANDO: "AVANZANDO HACIA PUERTA",
    ESTADO_POST_PUERTA: "AVANCE EXTRA POST-PUERTA",
    ESTADO_ATERRIZANDO: "ATERRIZANDO",
    ESTADO_FINALIZADO: "FINALIZADO"
}

class TelloAutonomousNode(Node):
    def __init__(self):
        super().__init__('tello_autonomous_controller_rgb_publisher')
        self.get_logger().info("Iniciando Nodo Autónomo Tello con Publicación de Imagen RGB...")

        self.tello = Tello(host=TELLO_IP)
        self.bridge = CvBridge()
        self.frame_reader = None
        self.tello_connected = False
        self.stream_on = False
        
        self.evento_parada_usuario = threading.Event()
        self.emergencia_activada = False
        
        self.estado_actual_mision = ESTADO_INICIO
        self.objetivo_perdido_contador = 0
        self.tiempo_inicio_post_puerta = 0
        self.tiempo_inicio_despegue = 0

        self.image_publisher = None

        try:
            self._init_tello()
            self._init_ros_publisher()
            self._init_keyboard_listener()
            
            self.main_timer = self.create_timer(MAIN_TIMER_PERIOD, self.main_loop_callback)
            self.get_logger().info("Nodo Tello inicializado y temporizador principal creado.")
            self.cambiar_estado_mision(ESTADO_DESPEGANDO)
            self.tiempo_inicio_despegue = time.time()
            self.tello.takeoff()
            self.get_logger().info("Comando de despegue enviado.")

        except Exception as e:
            self.get_logger().fatal(f"Error fatal durante la inicialización del nodo: {e}")
            traceback.print_exc()
            self.cleanup()
            rclpy.shutdown() # Asegurarse de que rclpy se cierre si el nodo falla al inicio

    def _init_tello(self):
        self.get_logger().info(f"Conectando al Tello en {TELLO_IP}...")
        self.tello.connect()
        self.tello_connected = True
        self.get_logger().info(f"Conectado. Batería: {self.tello.get_battery()}%")

        if self.tello.get_battery() < 15:
            raise RuntimeError(f"Batería baja ({self.tello.get_battery()}%). Abortando.")

        self.get_logger().info("Iniciando stream de vídeo...")
        self.tello.streamoff()
        time.sleep(0.2)
        self.tello.streamon()
        self.stream_on = True
        self.frame_reader = self.tello.get_frame_read()
        if self.frame_reader is None:
            raise RuntimeError("No se pudo obtener el frame_reader del Tello.")
        self.get_logger().info("Stream activado y frame_reader obtenido.")
        time.sleep(0.5)

    def _init_ros_publisher(self):
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1 
        )
        self.image_publisher = self.create_publisher(Image, ROS_IMAGE_TOPIC_OUTPUT, qos_profile)
        self.get_logger().info(f"Publicador ROS creado para el topic: {ROS_IMAGE_TOPIC_OUTPUT}")

    def _init_keyboard_listener(self):
        self.keyboard_listener_thread = threading.Thread(target=self._keyboard_listener_loop, daemon=True)
        self.keyboard_listener_thread.start()
        self.get_logger().info("Hilo de escucha de teclado iniciado.")

    def _keyboard_listener_loop(self):
        self.get_logger().info("[Teclado] Hilo iniciado. 'q' para aterrizar/salir, 'w' para EMERGENCIA.")
        def on_press(key):
            try:
                if key.char == 'q':
                    if not self.evento_parada_usuario.is_set() and not self.emergencia_activada:
                        self.get_logger().info("[Teclado] 'q' presionada. Solicitando aterrizaje y parada...")
                        self.evento_parada_usuario.set()
                elif key.char == 'w':
                    if not self.emergencia_activada:
                        self.get_logger().warn("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                        self.get_logger().warn("[Teclado] 'w' presionada. ¡¡¡PARADA DE EMERGENCIA!!!")
                        self.get_logger().warn("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                        self.emergencia_activada = True
                        if self.tello_connected:
                            try:
                                self.tello.emergency()
                            except Exception as e_emerg:
                                self.get_logger().error(f"Error al enviar comando de emergencia: {e_emerg}")
                        self.evento_parada_usuario.set() 
            except AttributeError:
                pass
            except Exception as e:
                self.get_logger().error(f"[Teclado] Error en on_press: {e}")

        with keyboard.Listener(on_press=on_press) as listener:
            try:
                listener.join()
            except Exception as e:
                 self.get_logger().error(f"[Teclado] Excepción en listener.join(): {e}")
        self.get_logger().info("[Teclado] Hilo finalizado.")
        
    def cambiar_estado_mision(self, nuevo_estado, mensaje_log=""):
        if self.estado_actual_mision != nuevo_estado:
            self.get_logger().info(f"ESTADO: {estado_nombres.get(self.estado_actual_mision, 'DESCONOCIDO')} -> {estado_nombres.get(nuevo_estado, 'DESCONOCIDO')}. {mensaje_log}")
            self.estado_actual_mision = nuevo_estado
            if nuevo_estado == ESTADO_AVANZANDO:
                self.objetivo_perdido_contador = 0

    def procesar_y_controlar(self, frame_bgr):
        lr, fb, ud, yv = 0, 0, 0, VELOCIDAD_YAW

        img_proc_bgr = cv2.resize(frame_bgr, (FRAME_WIDTH_PROC, FRAME_HEIGHT_PROC))
        frame_center_x = FRAME_WIDTH_PROC // 2
        frame_center_y = FRAME_HEIGHT_PROC // 2

        hsv = cv2.cvtColor(img_proc_bgr, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, COLOR_LOWER_1, COLOR_UPPER_1)
        mask2 = cv2.inRange(hsv, COLOR_LOWER_2, COLOR_UPPER_2)
        mask_red = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        puerta_detectada_este_frame = False
        offset_x_px = 0
        offset_y_px = 0

        if contours:
            main_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(main_contour) > MIN_CONTOUR_AREA_TELLO:
                puerta_detectada_este_frame = True
                M = cv2.moments(main_contour)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.drawContours(img_proc_bgr, [main_contour], -1, (0, 255, 0), 2)
                    cv2.circle(img_proc_bgr, (cx, cy), 5, (0, 0, 255), -1)
                    
                    offset_x_px = cx - frame_center_x
                    offset_y_px = cy - frame_center_y
                else:
                    puerta_detectada_este_frame = False
        
        if self.estado_actual_mision == ESTADO_DESPEGANDO:
            if time.time() - self.tiempo_inicio_despegue > 3.0: # Esperar 3s para estabilizar
                self.cambiar_estado_mision(ESTADO_BUSCANDO_ALINEANDO, "Despegue estabilizado.")
            else:
                 self.get_logger().info("Despegando y estabilizando...", throttle_duration_sec=1.0)


        elif self.estado_actual_mision == ESTADO_BUSCANDO_ALINEANDO:
            if puerta_detectada_este_frame:
                self.get_logger().debug(f"BUSCANDO/ALINEANDO: Puerta detectada. Offset X: {offset_x_px}, Offset Y: {offset_y_px}")
                if abs(offset_x_px) > MARGEN_ERROR_X_PIXELS:
                    lr = -VELOCIDAD_CORRECCION if offset_x_px < 0 else VELOCIDAD_CORRECCION
                if abs(offset_y_px) > MARGEN_ERROR_Y_PIXELS:
                    ud = -VELOCIDAD_CORRECCION if offset_y_px > 0 else VELOCIDAD_CORRECCION
                if abs(offset_x_px) <= MARGEN_ERROR_X_PIXELS and abs(offset_y_px) <= MARGEN_ERROR_Y_PIXELS:
                    self.cambiar_estado_mision(ESTADO_AVANZANDO, "¡Alineado con la puerta!")
                    lr, ud = 0, 0 
            else:
                self.get_logger().debug("BUSCANDO/ALINEANDO: No se detecta puerta. Manteniendo posición.")
                lr, fb, ud, yv = 0, 0, 0, 0

        elif self.estado_actual_mision == ESTADO_AVANZANDO:
            if puerta_detectada_este_frame:
                self.objetivo_perdido_contador = 0
                self.get_logger().debug(f"AVANZANDO: Hacia la puerta. Offset X: {offset_x_px}, Offset Y: {offset_y_px}")
                fb = VELOCIDAD_AVANCE
                if abs(offset_x_px) > MARGEN_ERROR_X_PIXELS:
                    lr = -VELOCIDAD_CORRECCION if offset_x_px < 0 else VELOCIDAD_CORRECCION
                if abs(offset_y_px) > MARGEN_ERROR_Y_PIXELS:
                    ud = -VELOCIDAD_CORRECCION if offset_y_px > 0 else VELOCIDAD_CORRECCION
            else:
                self.objetivo_perdido_contador += 1
                self.get_logger().info(f"AVANZANDO: Puerta perdida ({self.objetivo_perdido_contador}/{CONTADOR_PERDIDO_MAX}). Manteniendo avance.")
                fb = VELOCIDAD_AVANCE 
                if self.objetivo_perdido_contador > CONTADOR_PERDIDO_MAX:
                    self.cambiar_estado_mision(ESTADO_POST_PUERTA, "Se asume que la puerta fue pasada.")
                    self.tiempo_inicio_post_puerta = time.time()
                    fb = 0 
        
        elif self.estado_actual_mision == ESTADO_POST_PUERTA:
            self.get_logger().info("POST_PUERTA: Realizando avance extra...", throttle_duration_sec=0.5)
            fb = VELOCIDAD_AVANCE
            lr, ud, yv = 0,0,0
            if time.time() - self.tiempo_inicio_post_puerta > TIEMPO_AVANCE_EXTRA_SEG:
                self.cambiar_estado_mision(ESTADO_ATERRIZANDO, "Avance extra completado.")
                fb = 0

        elif self.estado_actual_mision == ESTADO_ATERRIZANDO:
            self.get_logger().info("ATERRIZANDO: Preparando para aterrizar.")
            lr, fb, ud, yv = 0, 0, 0, 0
            if self.tello_connected and self.tello.is_flying:
                try:
                    self.tello.land()
                except Exception as e:
                    self.get_logger().error(f"Error al intentar aterrizar: {e}")
            self.cambiar_estado_mision(ESTADO_FINALIZADO, "Aterrizaje comandado.")

        elif self.estado_actual_mision == ESTADO_FINALIZADO:
            lr, fb, ud, yv = 0, 0, 0, 0
            pass

        if self.estado_actual_mision not in [ESTADO_FINALIZADO, ESTADO_INICIO, ESTADO_DESPEGANDO] and \
           not self.emergencia_activada and self.tello_connected and self.tello.is_flying:
            try:
                self.tello.send_rc_control(lr, fb, ud, yv)
            except Exception as e:
                self.get_logger().error(f"Error enviando RC control: {e}", throttle_duration_sec=1.0)
        
        return img_proc_bgr 


    def main_loop_callback(self):
        if self.evento_parada_usuario.is_set() or self.estado_actual_mision == ESTADO_FINALIZADO:
            if self.estado_actual_mision != ESTADO_FINALIZADO and self.estado_actual_mision != ESTADO_ATERRIZANDO:
                 self.cambiar_estado_mision(ESTADO_ATERRIZANDO, "Parada solicitada, iniciando aterrizaje.")
            # Si ya está aterrizando o finalizado por otra razón, o se pidió parada, eventualmente parará rclpy
            if self.estado_actual_mision == ESTADO_FINALIZADO and rclpy.ok():
                self.get_logger().info("Misión finalizada o parada, deteniendo rclpy.")
                # self.cleanup() # cleanup se llama en destroy_node
                rclpy.try_shutdown() # Intenta cerrar rclpy limpiamente
            return

        if self.emergencia_activada:
            self.get_logger().warn("EMERGENCIA ACTIVADA. No se procesarán más frames.")
            if rclpy.ok(): rclpy.try_shutdown()
            return

        if not self.tello_connected or self.frame_reader is None or self.frame_reader.stopped:
            self.get_logger().error("Tello no conectado o stream detenido. Intentando cerrar.", throttle_duration_sec=5.0)
            self.evento_parada_usuario.set()
            if rclpy.ok(): rclpy.try_shutdown()
            return

        frame_bgr_raw = self.frame_reader.frame
        if frame_bgr_raw is None:
            self.get_logger().warn("Frame del Tello es None.", throttle_duration_sec=2.0)
            return

        try:
            img_procesada_bgr = self.procesar_y_controlar(frame_bgr_raw)

            if img_procesada_bgr is not None and self.image_publisher is not None:
                img_procesada_rgb = cv2.cvtColor(img_procesada_bgr, cv2.COLOR_BGR2RGB)
                ros_image_msg = self.bridge.cv2_to_imgmsg(img_procesada_rgb, encoding="rgb8")
                ros_image_msg.header.stamp = self.get_clock().now().to_msg()
                ros_image_msg.header.frame_id = "tello_camera_processed"
                self.image_publisher.publish(ros_image_msg)

        except Exception as e:
            self.get_logger().error(f"Error en main_loop_callback: {e}")
            traceback.print_exc()
            self.evento_parada_usuario.set() # Intentar parar en caso de error grave
            if rclpy.ok(): rclpy.try_shutdown()


    def cleanup(self):
        self.get_logger().info("--- Iniciando Limpieza del Nodo Tello ---")
        
        if hasattr(self, 'main_timer') and self.main_timer:
            self.main_timer.cancel()
            self.get_logger().info("Temporizador principal cancelado.")

        if hasattr(self, 'keyboard_listener_thread') and self.keyboard_listener_thread.is_alive():
            self.get_logger().info("Deteniendo listener de teclado (puede tardar si está bloqueado)...")
            # pynput listener.stop() debe ser llamado desde el mismo hilo del listener
            # o usar un evento. Aquí el evento_parada_usuario debería ayudar a que salga.
            # Para daemon threads, simplemente permitir que el programa principal termine.

        if self.tello_connected:
            if self.emergencia_activada:
                self.get_logger().info("EMERGENCIA FUE ACTIVADA. El dron debería haberse detenido.")
            elif self.estado_actual_mision not in [ESTADO_FINALIZADO, ESTADO_ATERRIZANDO] or self.tello.is_flying :
                self.get_logger().info("Intentando aterrizar el dron como medida de seguridad final...")
                try:
                    if self.tello.is_flying:
                        self.tello.send_rc_control(0,0,0,0)
                        time.sleep(0.1)
                        self.tello.land()
                        time.sleep(3)
                except Exception as land_err:
                    self.get_logger().error(f"Error durante el aterrizaje en cleanup: {land_err}")
            
            if self.stream_on:
                self.get_logger().info("Deteniendo stream de vídeo...")
                try:
                    self.tello.streamoff()
                except Exception as e_so: self.get_logger().error(f"Error en streamoff: {e_so}")
            
            self.get_logger().info("Desconectando del Tello...")
            try:
                self.tello.end()
            except Exception as e_end: self.get_logger().error(f"Error en tello.end(): {e_end}")
        
        self.tello_connected = False
        self.stream_on = False
        self.get_logger().info("--- Limpieza del Nodo Tello Finalizada ---")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TelloAutonomousNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info("Ctrl+C detectado, iniciando cierre del nodo...")
        else: print("Ctrl+C detectado antes de crear nodo.")
    except SystemExit:
        if node: node.get_logger().info("SystemExit, probablemente por error de inicialización.")
        else: print("SystemExit antes de crear nodo.")
    except Exception as e:
        if node: node.get_logger().fatal(f"Excepción no controlada en main: {e}")
        else: print(f"Excepción no controlada en main antes de crear nodo: {e}")
        traceback.print_exc()
    finally:
        if node:
            node.get_logger().info("Bloque finally en main: Llamando a cleanup y destroy_node.")
            node.evento_parada_usuario.set() # Asegurar que el bucle se detenga
            node.cleanup() # Llamar a cleanup explícitamente
            if rclpy.ok() and node.is_valid(): # Solo destruir si el nodo es válido
                 node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Programa ROS 2 Tello finalizado.")

if __name__ == '__main__':
    main()
