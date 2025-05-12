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
import traceback

# --- Constantes ---
TELLO_IP = '192.168.10.1'
ROS_TOPIC_IMAGEN_RAW = '/tello/imagen_raw'
ROS_TOPIC_COMANDOS_VELOCIDAD = '/tello/comandos_velocidad'
TIMER_PERIODO_CAMARA = 1.0 / 30.0  # ~30 FPS para la cámara
FRAME_TIMEOUT_NONE_MAX = 150 # Frames None antes de considerar error grave

class NodoCamaraTello(Node):
    def __init__(self):
        super().__init__('nodo_camara_tello')
        self.get_logger().info(f"Iniciando Nodo Camara Tello. IP Tello: {TELLO_IP}")

        self.bridge = CvBridge()
        self.tello = Tello(host=TELLO_IP)
        self.tello_conectado = False
        self.stream_activo = False
        self.frame_reader = None
        self.parada_emergencia_solicitada = False
        self.aterrizaje_solicitado = False
        self.despegue_realizado = False
        self.ultimo_comando_velocidad = [0.0, 0.0, 0.0, 0.0] # lr, fb, ud, yv

        self.none_frame_streak = 0

        try:
            self.get_logger().info("Conectando al Tello...")
            self.tello.connect()
            self.tello_conectado = True
            self.get_logger().info(f"Conectado. Batería: {self.tello.get_battery()}%")

            if self.tello.get_battery() < 20:
                self.get_logger().error("¡¡¡BATERÍA BAJA!!! No se despegará. Cancela y carga la batería.")
                # No se levanta SystemExit para permitir que el nodo se cierre limpiamente
                # rclpy.shutdown() podría no ser seguro aquí si se llama desde __init__ antes de que el spin comience
                return # Salir del init si la batería es baja

            self.get_logger().info("Iniciando stream de vídeo...")
            self.tello.streamoff()
            time.sleep(0.5)
            self.tello.streamon()
            self.stream_activo = True
            self.frame_reader = self.tello.get_frame_read()
            if self.frame_reader is None:
                raise RuntimeError("frame_reader es None después de streamon().")
            self.get_logger().info("Stream activado y frame_reader obtenido.")
            time.sleep(1.0) # Pausa para estabilizar stream

            self.get_logger().info("¡DESPEGANDO! Mantén el área despejada.")
            # self.tello.takeoff() 
            self.get_logger().info("Despegue simulado completado (takeoff() está comentado). Altura actual: N/A (simulado)")
            # En un dron real, podrías esperar a que el estado de despegue se confirme
            # o usar tello.get_height() para verificar.
            self.despegue_realizado = True # Asumir despegue completado
            time.sleep(2) # Tiempo para estabilizarse tras despegar


        except Exception as e:
            self.get_logger().fatal(f"Error crítico inicializando Tello o despegando: {e}")
            self.get_logger().fatal(traceback.format_exc())
            self.cleanup_recursos()
            # Indicar que el nodo no debe continuar
            # Es mejor dejar que rclpy.spin falle si el nodo no está bien inicializado
            # o manejarlo en el main. Por ahora, marcaremos que no está listo.
            self.tello_conectado = False # Asegurar que no se intente usar si falló
            return


        # Publicador para la imagen RAW
        qos_profile_img = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1 # Solo el último frame es relevante
        )
        self.publicador_imagen_raw = self.create_publisher(Image, ROS_TOPIC_IMAGEN_RAW, qos_profile_img)
        self.get_logger().info(f"Publicando imagen RAW en: {ROS_TOPIC_IMAGEN_RAW}")

        # Suscriptor para los comandos de velocidad
        qos_profile_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, # Los comandos deben llegar
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

        # Hilo para monitorizar teclado
        self.hilo_teclado = threading.Thread(target=self.monitor_teclado, daemon=True)
        self.hilo_teclado.start()

    def monitor_teclado(self):
        self.get_logger().info("[Hilo Teclado] Iniciado. Escuchando 'f' (emergencia) y esperando Ctrl+C.")
        # pynput no detecta bien Ctrl+C como una tecla normal en todos los terminales
        # Ctrl+C se maneja mejor con la excepción KeyboardInterrupt en el hilo principal de rclpy.spin()
        def on_press(key):
            try:
                if key.char == 'f':
                    if not self.parada_emergencia_solicitada and not self.aterrizaje_solicitado:
                        self.get_logger().warn("Tecla 'f' presionada. ¡SOLICITANDO PARADA DE EMERGENCIA!")
                        self.parada_emergencia_solicitada = True
            except AttributeError: # Teclas especiales (Ctrl, Shift, etc.)
                pass

        with keyboard.Listener(on_press=on_press) as listener:
            try:
                listener.join()
            except Exception as e_listener:
                 self.get_logger().error(f"[Hilo Teclado] Error en listener.join: {e_listener}")
        self.get_logger().info("[Hilo Teclado] Finalizado.")


    def callback_comandos_velocidad(self, msg):
        if not self.tello_conectado or not self.despegue_realizado or self.parada_emergencia_solicitada or self.aterrizaje_solicitado:
            return # No procesar comandos si no estamos listos o en parada

        if len(msg.data) == 4:
            lr, fb, ud, yv = int(msg.data[0]), int(msg.data[1]), int(msg.data[2]), int(msg.data[3])
            self.ultimo_comando_velocidad = [lr, fb, ud, yv] # Guardar para posible parada suave
            try:
                self.tello.send_rc_control(lr, fb, ud, yv)
            except Exception as e:
                self.get_logger().error(f"Error enviando send_rc_control: {e}", throttle_duration_sec=2.0)
        else:
            self.get_logger().warn(f"Comando de velocidad recibido con longitud incorrecta: {len(msg.data)}", throttle_duration_sec=5.0)

    def timer_callback_camara(self):
        if not self.tello_conectado or not self.stream_activo or self.frame_reader is None:
            if self.tello_conectado: # Solo loguear si se supone que debería estar funcionando
                 self.get_logger().warn("Tello conectado pero frame_reader no disponible o stream no activo.", throttle_duration_sec=5.0)
            return

        if self.parada_emergencia_solicitada or self.aterrizaje_solicitado:
            # Si se solicitó parada o aterrizaje, no seguir publicando frames ni enviando RC
            # El manejo de emergencia/aterrizaje se hace en destroy_node o main
            return

        if self.frame_reader.stopped:
            self.get_logger().error("El lector de frames del Tello se ha detenido. Solicitando aterrizaje.")
            self.aterrizaje_solicitado = True # Marcar para que cleanup actúe
            # Esto debería ser manejado por el bucle principal de rclpy para llamar a destroy_node
            rclpy.shutdown() # Forzar cierre si el lector de frames muere
            return

        frame_bgr = self.frame_reader.frame

        if frame_bgr is None:
            self.none_frame_streak += 1
            if self.none_frame_streak == 1 or self.none_frame_streak % 60 == 0: # Loguear menos frecuentemente
                self.get_logger().warn(f"Frame es None (Streak: {self.none_frame_streak}). Lector vivo: {self.frame_reader.thread.is_alive() if hasattr(self.frame_reader, 'thread') and self.frame_reader.thread else 'N/A'}", throttle_duration_sec=5.0)
            if self.none_frame_streak > FRAME_TIMEOUT_NONE_MAX:
                self.get_logger().error(f"Timeout por {FRAME_TIMEOUT_NONE_MAX} frames None. Solicitando aterrizaje.")
                self.aterrizaje_solicitado = True
                rclpy.shutdown()
            return

        self.none_frame_streak = 0 # Resetear si el frame es válido

        try:
            # El frame del Tello ya es BGR, que es común para OpenCV.
            # Para ROS, Image espera BGR8, RGB8, etc. Si es BGR, publicamos como BGR8.
            ros_image_msg = self.bridge.cv2_to_imgmsg(frame_bgr, encoding="bgr8")
            ros_image_msg.header.stamp = self.get_clock().now().to_msg()
            ros_image_msg.header.frame_id = "tello_camera_link_raw" # Identificador del frame
            self.publicador_imagen_raw.publish(ros_image_msg)
        except CvBridgeError as e_cv_bridge:
            self.get_logger().error(f"Error CvBridge al convertir/publicar imagen RAW: {e_cv_bridge}", throttle_duration_sec=5.0)
        except Exception as e_publish:
            self.get_logger().error(f"Error publicando imagen RAW: {e_publish}", throttle_duration_sec=5.0)


    def cleanup_recursos(self, aterrizar_normalmente=True):
        self.get_logger().info("Iniciando limpieza de recursos del NodoCamaraTello...")

        # Detener timers
        if hasattr(self, 'timer_camara') and self.timer_camara:
            self.timer_camara.cancel()
            self.get_logger().info("Timer de cámara cancelado.")

        if self.tello_conectado:
            try:
                # Enviar comando de parada de movimiento por si acaso
                self.get_logger().info("Enviando comando de detención (RC 0,0,0,0)...")
                self.tello.send_rc_control(0, 0, 0, 0)
                time.sleep(0.1) # Pequeña pausa para que el comando se procese
            except Exception as e_rc_stop:
                self.get_logger().error(f"Excepción enviando RC stop: {e_rc_stop}")

            if self.parada_emergencia_solicitada:
                self.get_logger().warn("¡EJECUTANDO PARADA DE EMERGENCIA DEL TELLO!")
                try:
                    self.tello.emergency() # Corta motores inmediatamente
                    self.get_logger().info("Comando de emergencia enviado.")
                except Exception as e_emergency:
                    self.get_logger().error(f"Excepción durante tello.emergency(): {e_emergency}")
                # Después de emergency(), el dron podría no responder a más comandos.

            elif self.despegue_realizado and aterrizar_normalmente : # Solo aterrizar si despegó y no fue emergencia
                self.get_logger().info("Intentando aterrizar el Tello...")
                try:
                    # self.tello.land() # *** DESCOMENTAR PARA ATERRIZAJE REAL ***
                    self.get_logger().info("Comando de aterrizaje (land()) SIMULADO.")
                    # En un dron real, esperarías a que el aterrizaje se complete.
                    # time.sleep(5) # Espera simulada para aterrizaje
                except Exception as e_land:
                    self.get_logger().error(f"Excepción durante tello.land(): {e_land}")

            if self.stream_activo:
                self.get_logger().info("Deteniendo stream de vídeo del Tello...")
                try:
                    self.tello.streamoff()
                except Exception as e_streamoff:
                    self.get_logger().error(f"Excepción durante tello.streamoff(): {e_streamoff}")
                self.stream_activo = False

            self.get_logger().info("Desconectando del Tello...")
            try:
                self.tello.end()
            except Exception as e_end:
                self.get_logger().error(f"Excepción durante tello.end(): {e_end}")
            self.tello_conectado = False
        else:
            self.get_logger().info("Tello no estaba conectado, saltando limpieza de Tello.")

        self.get_logger().info("Limpieza de recursos de NodoCamaraTello finalizada.")

    def destroy_node(self):
        self.get_logger().info("Destruyendo NodoCamaraTello...")
        # Si KeyboardInterrupt causó la salida, aterrizaje_solicitado podría no estar True
        # pero queremos aterrizar de todas formas si no fue una emergencia.
        self.cleanup_recursos(aterrizar_normalmente=not self.parada_emergencia_solicitada)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    nodo_camara = None
    try:
        nodo_camara = NodoCamaraTello()
        if not nodo_camara.tello_conectado and not nodo_camara.parada_emergencia_solicitada: # Si falló la conexión inicial y no fue por batería baja que permite cierre limpio
             # O si falló el despegue (si se implementa chequeo)
            raise RuntimeError("Fallo en la inicialización del nodo cámara, no se puede continuar.")

        if nodo_camara.tello.get_battery() < 20 and nodo_camara.tello_conectado: # Doble chequeo por si no salió del init
            nodo_camara.get_logger().error("Batería baja detectada después de __init__. Abortando antes de spin.")
            # No es necesario rclpy.shutdown() aquí, el finally lo hará.
        elif rclpy.ok(): # Solo girar si todo está bien
             nodo_camara.get_logger().info("NodoCamaraTello listo y entrando en bucle de spin.")
             rclpy.spin(nodo_camara)

    except KeyboardInterrupt:
        if nodo_camara:
            nodo_camara.get_logger().info("Ctrl+C detectado. Solicitando aterrizaje normal.")
            nodo_camara.aterrizaje_solicitado = True # Para que cleanup_recursos sepa que es un aterrizaje normal
        else:
            print("Ctrl+C detectado antes de inicializar el nodo completamente.")
    except RuntimeError as e_runtime:
        if nodo_camara:
            nodo_camara.get_logger().fatal(f"Error de ejecución crítico: {e_runtime}")
        else:
            print(f"Error de ejecución crítico antes de inicializar el nodo: {e_runtime}")
    except Exception as e_main:
        if nodo_camara:
            nodo_camara.get_logger().fatal(f"Error inesperado en main de NodoCamaraTello: {e_main}")
            nodo_camara.get_logger().fatal(traceback.format_exc())
        else:
            print(f"Error inesperado en main antes de inicializar el nodo: {e_main}")
            print(traceback.format_exc())
    finally:
        if nodo_camara:
            # destroy_node() es llamado automáticamente por rclpy.shutdown() o al salir del spin
            # pero nos aseguramos de que se marque para aterrizaje si no es emergencia
            if not nodo_camara.parada_emergencia_solicitada:
                nodo_camara.aterrizaje_solicitado = True
            # No es necesario llamar a nodo_camara.destroy_node() explícitamente aquí si rclpy.shutdown() se va a llamar
        if rclpy.ok():
            rclpy.shutdown()
        print("Programa NodoCamaraTello finalizado.")

if __name__ == '__main__':
    main()