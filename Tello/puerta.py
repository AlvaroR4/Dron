# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from djitellopy import Tello
import cv2
import numpy as np
import time
import traceback
import math

TELLO_IP = '192.168.10.1'
ROS_IMAGE_TOPIC_OUTPUT = '/tello/image_processed_rgb'
MAIN_TIMER_PERIOD = 1.0 / 20.0

FRAME_WIDTH_PROC = 640
FRAME_HEIGHT_PROC = 480
MIN_CONTOUR_AREA_TELLO = 20

TAMANO_REAL_PUERTA_M = 0.5
DISTANCIA_FOCAL_PIXELS_TELLO = 920

COLOR_LOWER_1 = np.array([0, 130, 90])
COLOR_UPPER_1 = np.array([10, 255, 255])
COLOR_LOWER_2 = np.array([160, 130, 90])
COLOR_UPPER_2 = np.array([179, 255, 255])

VELOCIDAD_AVANCE = 25
VELOCIDAD_CORRECCION = 30
VELOCIDAD_YAW = 0
MARGEN_ERROR_X_PIXELS = 0.1
MARGEN_ERROR_Y_PIXELS = 0.1
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

        self.estado_actual_mision = ESTADO_INICIO
        self.objetivo_perdido_contador = 0
        self.tiempo_inicio_post_puerta = 0
        self.tiempo_inicio_despegue = 0
        qos_profile_display = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # BEST_EFFORT es común para vídeo
            history=HistoryPolicy.KEEP_LAST,
            depth=5  # Mantener pocos mensajes si el subscriber es lento
        )

        self.display_publisher_ = self.create_publisher(Image, ROS_IMAGE_TOPIC_OUTPUT, qos_profile_display)

        self.image_publisher = None

        try:
            self._init_tello()

            self.main_timer = self.create_timer(MAIN_TIMER_PERIOD, self.main_loop_callback)
            self.get_logger().info("Nodo Tello inicializado y temporizador principal creado.")
            self.cambiar_estado_mision(ESTADO_DESPEGANDO)
            self.tiempo_inicio_despegue = time.time()
            #self.tello.takeoff()
            self.get_logger().info("Comando de despegue enviado.")

        except Exception as e:
            self.get_logger().fatal(f"Error fatal durante la inicialización del nodo: {e}")
            traceback.print_exc()
            self.cleanup()
            if rclpy.ok():
                rclpy.shutdown()

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

    def cambiar_estado_mision(self, nuevo_estado, mensaje_log=""):
        if self.estado_actual_mision != nuevo_estado:
            self.get_logger().info(f"ESTADO: {estado_nombres.get(self.estado_actual_mision, 'DESCONOCIDO')} -> {estado_nombres.get(nuevo_estado, 'DESCONOCIDO')}. {mensaje_log}")
            self.estado_actual_mision = nuevo_estado
            if nuevo_estado == ESTADO_AVANZANDO:
                self.objetivo_perdido_contador = 0

    def calcular_distancia(self, tamanio_aparente_pixels, distancia_focal_pixels, tamano_real_objeto_m):
        if tamanio_aparente_pixels <= 1:
            return float('inf')
        return (tamano_real_objeto_m * distancia_focal_pixels) / tamanio_aparente_pixels

    def procesar_y_controlar(self, frame_bgr_raw):
        lr, fb, ud, yv = 0, 0, 0, 0
        puerta_detectada_este_frame = False
        offset_x_px = 0
        offset_y_px = 0
        distancia_m = float('inf')
        ##COMIENZO

        if self.frame_reader is None or not self.tello_connected:
            self.get_logger().warn("frame_reader no disponible o Tello no conectado en timer_callback.", throttle_duration_sec=5.0)
            return
        
        if self.frame_reader.stopped:
            self.get_logger().error("El lector de frames del Tello se ha detenido. Cerrando nodo.")
            self.cleanup_tello() # Llama a la función de limpieza
            rclpy.shutdown()
            return

        current_frame_bgr_raw = self.frame_reader.frame # El frame del Tello es BGR

        if current_frame_bgr_raw is None:
            self.none_frame_streak += 1
            # Loguear con menos frecuencia para no inundar la consola
            if self.none_frame_streak == 1 or self.none_frame_streak % 60 == 0:
                 is_thread_alive_str = 'N/A'
                 if hasattr(self.frame_reader, 'thread') and self.frame_reader.thread is not None:
                     is_thread_alive_str = str(self.frame_reader.thread.is_alive())
                 self.get_logger().warn(f"Frame es None (Streak: {self.none_frame_streak}). Lector vivo: {is_thread_alive_str}", throttle_duration_sec=5.0)
            
            if self.none_frame_streak > 20:
                 self.get_logger().error(f"Timeout por 20 frames None consecutivos. Deteniendo nodo.")
                 self.cleanup_tello() # Llama a la función de limpieza
                 rclpy.shutdown()
                 return
            return # Salir del callback si no hay frame
        
        # Si llegamos aquí, el frame es válido
        self.none_frame_streak = 0
        #self.frame_counter_validos += 1

        try:
            # --- Procesamiento de Imagen ---
            # Redimensionar el frame BGR original
            img_proc_bgr = cv2.resize(current_frame_bgr_raw, (FRAME_WIDTH_PROC, FRAME_HEIGHT_PROC))
            # Crear una copia para dibujar sobre ella (aún en BGR)
            img_proc = cv2.cvtColor(img_proc_bgr, cv2.COLOR_BGR2RGB)

            # 1. Segmentación de Color Rojo usando HSV
            # Convertir la imagen de trabajo BGR a HSV
            img_hsv = cv2.cvtColor(img_proc, cv2.COLOR_BGR2HSV)
            
            # Aplicar máscaras para el color rojo
            mask1_red = cv2.inRange(img_hsv, COLOR_LOWER_1, COLOR_UPPER_1)
            mask2_red = cv2.inRange(img_hsv, COLOR_LOWER_2, COLOR_UPPER_2)
            final_mask_red = cv2.bitwise_or(mask1_red, mask2_red)
            
            # Opcional: Aplicar operaciones morfológicas para limpiar la máscara
            # kernel_morph = np.ones((3,3),np.uint8) # Kernel más pequeño
            # final_mask_red = cv2.morphologyEx(final_mask_red, cv2.MORPH_OPEN, kernel_morph, iterations=1)
            # final_mask_red = cv2.morphologyEx(final_mask_red, cv2.MORPH_CLOSE, kernel_morph, iterations=2)

            # 2. Encontrar Contornos en la máscara final
            contours, _ = cv2.findContours(final_mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # 3. Análisis de Contornos para encontrar el "marco" o "puerta"
            detected_targets_info = []
            frame_center_x = FRAME_WIDTH_PROC // 2
            # frame_center_y = FRAME_HEIGHT_PROC // 2 # No se usa offset_y en UDP, pero útil para logs

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > MIN_CONTOUR_AREA_TELLO:
                    # Calcular momentos para obtener el centroide
                    M = cv2.moments(cnt)
                    if M['m00'] > 0: # Evitar división por cero
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        offset_x_pixels = cx - frame_center_x
                        offset_y_pixels = cy - (FRAME_HEIGHT_PROC // 2) # Centro Y del frame

                        # Usar minAreaRect para obtener un rectángulo que encierre el contorno
                        # Esto puede ser más robusto para la estimación de tamaño/distancia que boundingRect
                        rect_min_area = cv2.minAreaRect(cnt)
                        # El tamaño del rectángulo es rect_min_area[1] = (ancho, alto)
                        # Usamos el mayor de los dos (o el que corresponda a TAMANO_REAL_PUERTA_M)
                        # Para una puerta, usualmente el alto es más consistente o el ancho si es cuadrado
                        (box_w, box_h) = rect_min_area[1]
                        # Decidir si usar ancho o alto para 'tamanio_aparente_pixels'
                        # Si TAMANO_REAL_PUERTA_M es la altura, usar box_h. Si es ancho, box_w.
                        # Asumamos que TAMANO_REAL_PUERTA_M se refiere a la altura para este ejemplo.
                        tamanio_aparente_pixels_para_dist = max(box_w, box_h) # O min(), o uno específico. ¡Ajustar!
                        
                        distancia_m = self.calcular_distancia(tamanio_aparente_pixels_para_dist,
                                                         DISTANCIA_FOCAL_PIXELS_TELLO,
                                                         TAMANO_REAL_PUERTA_M)
                        
                        detected_targets_info.append({
                            'cx': cx, 'cy': cy, 
                            'offset_x': offset_x_pixels, 'offset_y': offset_y_pixels, # Guardamos ambos offsets
                            'distance': distancia_m, 
                            'area': area, 
                            'rect_obj': rect_min_area # Guardamos el objeto rectángulo para dibujar
                        })

            # 4. Selección del Mejor Objetivo (el más cercano) y Preparación de Datos UDP
            udp_offset_x_norm, udp_offset_y_norm, udp_distancia_m = 0.0, 0.0, float('inf')
            num_valid_targets = len(detected_targets_info)

            if num_valid_targets > 0:
                # Ordenar los objetivos detectados por distancia (el más cercano primero)
                detected_targets_info.sort(key=lambda t: t['distance'])
                closest_target = detected_targets_info[0]
                
                # Normalizar offset_x: (-1 a 1), donde -1 es extremo izquierdo, 1 extremo derecho
                udp_offset_x_norm = closest_target['offset_x'] / (FRAME_WIDTH_PROC / 2.0)
                # Normalizar offset_y: (-1 a 1), donde -1 es extremo superior, 1 extremo inferior
                udp_offset_y_norm = closest_target['offset_y'] / (FRAME_HEIGHT_PROC / 2.0)
                udp_distancia_m = closest_target['distance']

                # Dibujar el rectángulo y el centroide del objetivo más cercano en la imagen BGR
                points = cv2.boxPoints(closest_target['rect_obj'])
                points = np.intp(points) # Convertir a enteros para dibujar
                cv2.drawContours(img_proc, [points], -1, (0, 255, 0), 2) # Verde para el marco
                cv2.circle(img_proc, (closest_target['cx'], closest_target['cy']), 5, (0, 0, 255), -1) # Rojo para el centro
                
                info_text = f"D:{closest_target['distance']:.1f}m Ox:{udp_offset_x_norm:.2f} Oy:{udp_offset_y_norm:.2f}"
                cv2.putText(img_proc, info_text,
                            (closest_target['cx'] - 60, closest_target['cy'] - 20), # Posición del texto
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA) # Amarillo

            # 6. Preparar y Publicar Imagen Procesada en ROS (como RGB8)
            try:
                # Convertir la imagen BGR (con dibujos) a RGB para publicación
                img_publish_rgb = cv2.cvtColor(img_proc, cv2.COLOR_BGR2RGB)
                
                # Crear mensaje ROS desde la imagen RGB
                ros_image_msg_out = self.bridge.cv2_to_imgmsg(img_publish_rgb, encoding="rgb8")
                ros_image_msg_out.header.stamp = self.get_clock().now().to_msg()
                ros_image_msg_out.header.frame_id = "tello_camera_processed_rgb" # Identificador del frame
                self.display_publisher_.publish(ros_image_msg_out)
            except CvBridgeError as e_cv_bridge:
                 self.get_logger().error(f"Error CvBridge al convertir/publicar imagen: {e_cv_bridge}")
            except Exception as e_publish_general:
                 self.get_logger().error(f"Error general al publicar visualización: {e_publish_general}")

        except Exception as e_processing_callback:
            self.get_logger().error(f"Error general en procesamiento del timer_callback: {e_processing_callback}")
            traceback.print_exc()


        #FIN

        if self.estado_actual_mision == ESTADO_DESPEGANDO:
            if time.time() - self.tiempo_inicio_despegue > 4.0:
                self.cambiar_estado_mision(ESTADO_BUSCANDO_ALINEANDO, "Despegue estabilizado.")
            else:
                 self.get_logger().info("Despegando y estabilizando...", throttle_duration_sec=1.0)
                 lr, fb, ud, yv = 0, 0, 0, 0

        elif self.estado_actual_mision == ESTADO_BUSCANDO_ALINEANDO:
            if num_valid_targets > 0:
                self.get_logger().debug(f"BUSCANDO/ALINEANDO: Puerta detectada. Offset X: {offset_x_px}, Offset Y: {offset_y_px}")
                if abs(offset_x_px) > MARGEN_ERROR_X_PIXELS:
                    lr = VELOCIDAD_CORRECCION if offset_x_px > 0 else -VELOCIDAD_CORRECCION
                else:
                    lr = 0

                if abs(offset_y_px) > MARGEN_ERROR_Y_PIXELS:
                    ud = VELOCIDAD_CORRECCION if offset_y_px > 0 else -VELOCIDAD_CORRECCION
                else:
                    ud = 0

                if lr == 0 and ud == 0:
                    self.cambiar_estado_mision(ESTADO_AVANZANDO, "¡Alineado con la puerta!")
                    lr, fb, ud, yv = 0, 0, 0, 0
            else:
                self.get_logger().debug("BUSCANDO/ALINEANDO: No se detecta puerta. Manteniendo posición.")
                lr, fb, ud, yv = 0, 0, 0, 0

        elif self.estado_actual_mision == ESTADO_AVANZANDO:
            if num_valid_targets > 0:
                self.objetivo_perdido_contador = 0
                self.get_logger().debug(f"AVANZANDO: Hacia la puerta. Offset X: {offset_x_px}, Offset Y: {offset_y_px}, Dist: {distancia_m:.1f}m")
                fb = VELOCIDAD_AVANCE

                if abs(offset_x_px) > MARGEN_ERROR_X_PIXELS:
                    lr = VELOCIDAD_CORRECCION if offset_x_px > 0 else -VELOCIDAD_CORRECCION
                else:
                    lr = 0
                if abs(offset_y_px) > MARGEN_ERROR_Y_PIXELS:
                    ud = VELOCIDAD_CORRECCION if offset_y_px > 0 else -VELOCIDAD_CORRECCION
                else:
                    ud = 0
            else:
                self.objetivo_perdido_contador += 1
                self.get_logger().info(f"AVANZANDO: Puerta perdida ({self.objetivo_perdido_contador}/{CONTADOR_PERDIDO_MAX}). Manteniendo avance...")
                fb = VELOCIDAD_AVANCE
                lr, ud, yv = 0, 0, 0

                if self.objetivo_perdido_contador > CONTADOR_PERDIDO_MAX:
                    self.cambiar_estado_mision(ESTADO_POST_PUERTA, "Se asume que la puerta fue pasada (contador perdido max).")
                    self.tiempo_inicio_post_puerta = time.time()
                    fb = 0

        elif self.estado_actual_mision == ESTADO_POST_PUERTA:
            self.get_logger().info("POST_PUERTA: Realizando avance extra...", throttle_duration_sec=0.5)
            fb = VELOCIDAD_AVANCE
            lr, ud, yv = 0, 0, 0
            if time.time() - self.tiempo_inicio_post_puerta > TIEMPO_AVANCE_EXTRA_SEG:
                self.cambiar_estado_mision(ESTADO_ATERRIZANDO, "Avance extra completado.")
                fb = 0

        elif self.estado_actual_mision == ESTADO_ATERRIZANDO:
            self.get_logger().info("ATERRIZANDO: Preparando para aterrizar.")
            lr, fb, ud, yv = 0, 0, 0, 0
            if self.tello_connected and self.tello.is_flying:
                try:
                    self.tello.land()
                    self.get_logger().info("Comando de aterrizaje enviado.")
                except Exception as e:
                    self.get_logger().error(f"Error al intentar aterrizar: {e}")
            self.cambiar_estado_mision(ESTADO_FINALIZADO, "Aterrizaje comandado.")

        elif self.estado_actual_mision == ESTADO_FINALIZADO:
            lr, fb, ud, yv = 0, 0, 0, 0
            pass

        if self.estado_actual_mision not in [ESTADO_FINALIZADO, ESTADO_INICIO, ESTADO_ATERRIZANDO] and \
           self.estado_actual_mision != ESTADO_DESPEGANDO and \
           self.tello_connected and self.tello.is_flying:
            try:
                lr_int, fb_int, ud_int, yv_int = int(lr), int(fb), int(ud), int(yv)
                print(f"DEBUG: Enviando RC -> Izq/Der:{lr_int}, Adel/Atras:{fb_int}, Arrib/Abaj:{ud_int}, Yaw:{yv_int}")
                #self.tello.send_rc_control(lr_int, fb_int, ud_int, yv_int)
            except Exception as e:
                self.get_logger().error(f"Error enviando RC control: {e}", throttle_duration_sec=1.0)
        elif self.estado_actual_mision == ESTADO_DESPEGANDO and self.tello_connected and self.tello.is_flying:
            try:
                 print("DEBUG: Enviando RC -> Izq/Der:0, Adel/Atras:0, Arrib/Abaj:0, Yaw:0 (Hover despegue)")
                 self.tello.send_rc_control(0, 0, 0, 0)
            except Exception as e:
                 self.get_logger().error(f"Error enviando RC control (hover despegue): {e}", throttle_duration_sec=1.0)

        return img_proc_bgr, img_proc


    def main_loop_callback(self):
        if self.estado_actual_mision == ESTADO_FINALIZADO:
            if rclpy.ok():
                self.get_logger().info("Misión finalizada, solicitando cierre de rclpy.", throttle_duration_sec=5.0)
                rclpy.try_shutdown()
            return

        if not self.tello_connected or self.frame_reader is None or self.frame_reader.stopped:
            self.get_logger().error("Tello no conectado o stream detenido. Iniciando parada.", throttle_duration_sec=5.0)
            # Intentar aterrizar directamente aquí si es posible y luego parar rclpy
            if self.tello_connected and self.tello.is_flying:
                try:
                    print("DEBUG: Enviando RC -> 0,0,0,0 (Stream perdido)")
                    self.tello.send_rc_control(0,0,0,0)
                    time.sleep(0.1)
                    print("DEBUG: Enviando land() (Stream perdido)")
                    self.tello.land()
                except Exception as e_land_lost:
                    self.get_logger().error(f"Error al aterrizar por stream perdido: {e_land_lost}")
            if rclpy.ok(): rclpy.try_shutdown()
            return

        frame_bgr_raw = self.frame_reader.frame
        if frame_bgr_raw is None:
            self.get_logger().warn("Frame del Tello es None. Esperando siguiente frame...", throttle_duration_sec=2.0)
            # Considerar detener el dron si no recibe frames por mucho tiempo
            # Por ejemplo, enviar RC 0,0,0,0
            if self.tello_connected and self.tello.is_flying:
                try:
                    print("DEBUG: Enviando RC -> 0,0,0,0 (Frame None)")
                    self.tello.send_rc_control(0,0,0,0)
                except Exception as e_rc_none:
                     self.get_logger().warn(f"Error enviando RC 0 por frame None: {e_rc_none}", throttle_duration_sec=5.0)
            return

        try:
            img_procesada_bgr, img_proc = self.procesar_y_controlar(frame_bgr_raw)
            """
            if img_procesada_bgr is not None and self.display_publisher_ is not None and self.display_publisher_.get_subscription_count() > 0:
                try:
                    img_publish_rgb = cv2.cvtColor(img_proc, cv2.COLOR_BGR2RGB)
                    # Crear mensaje ROS desde la imagen RGB
                    ros_image_msg_out = self.bridge.cv2_to_imgmsg(img_publish_rgb, encoding="rgb8")
                    ros_image_msg_out.header.stamp = self.get_clock().now().to_msg()
                    ros_image_msg_out.header.frame_id = "tello_camera_processed_rgb" # Identificador del frame
                    self.display_publisher_.publish(ros_image_msg_out)
                except CvBridgeError as e_bridge:
                    self.get_logger().error(f"CvBridge Error al publicar: {e_bridge}", throttle_duration_sec=5)
                except Exception as e_pub:
                    self.get_logger().error(f"Error general al publicar imagen: {e_pub}", throttle_duration_sec=5)
            """
        except Exception as e:
            self.get_logger().error(f"Error crítico en main_loop_callback (procesar/controlar/publicar): {e}")
            traceback.print_exc()
            # Intentar aterrizar en caso de error grave en el bucle
            if self.tello_connected and self.tello.is_flying:
                try:
                    print("DEBUG: Enviando RC -> 0,0,0,0 (Error en bucle)")
                    self.tello.send_rc_control(0,0,0,0)
                    time.sleep(0.1)
                    print("DEBUG: Enviando land() (Error en bucle)")
                    self.tello.land()
                except Exception as e_land_loop:
                    self.get_logger().error(f"Error al aterrizar por error en bucle: {e_land_loop}")
            if rclpy.ok(): rclpy.try_shutdown()


    def cleanup(self):
        self.get_logger().info("--- Iniciando Limpieza del Nodo Tello ---")

        if hasattr(self, 'main_timer') and self.main_timer:
            if not self.main_timer.is_canceled():
                self.main_timer.cancel()
                self.get_logger().info("Temporizador principal cancelado.")
            else:
                 self.get_logger().info("Temporizador principal ya estaba cancelado.")


        if self.tello_connected:
            if self.tello.is_flying and self.estado_actual_mision not in [ESTADO_ATERRIZANDO, ESTADO_FINALIZADO]:
                self.get_logger().warning("Cleanup: El dron parece seguir volando. Intentando aterrizar...")
                try:
                    print("DEBUG: Enviando RC -> 0,0,0,0 (Cleanup)")
                    self.tello.send_rc_control(0, 0, 0, 0)
                    time.sleep(0.2)
                    print("DEBUG: Enviando land() (Cleanup)")
                    self.tello.land()
                    self.get_logger().info("Comando land() enviado en cleanup.")
                    # time.sleep(4) # Evitar bloqueo largo en cleanup
                except Exception as land_err:
                    self.get_logger().error(f"Error durante el aterrizaje forzado en cleanup: {land_err}")
            else:
                 self.get_logger().info("Cleanup: El dron no está volando o ya se comandó aterrizaje. No se enviará land() adicional.")

            if self.stream_on:
                self.get_logger().info("Deteniendo stream de vídeo Tello...")
                try:
                    self.tello.streamoff()
                    self.stream_on = False
                except Exception as e_so:
                    self.get_logger().error(f"Excepción al intentar Tello.streamoff(): {e_so}")

            self.get_logger().info("Desconectando del Tello...")
            try:
                self.tello.end()
            except Exception as e_end:
                self.get_logger().error(f"Excepción al intentar Tello.end(): {e_end}")

        self.tello_connected = False
        self.stream_on = False

        self.get_logger().info("--- Limpieza del Nodo Tello Finalizada ---")


def main(args=None):
    print("Iniciando programa ROS 2 Tello Autónomo...")
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = TelloAutonomousNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nCtrl+C detectado. Intentando aterrizaje inmediato...")
        if node and node.tello_connected and node.tello.is_flying:
            try:
                print("DEBUG: Enviando RC -> Izq/Der:0, Adel/Atras:0, Arrib/Abaj:0, Yaw:0 (Parada por Ctrl+C)")
                node.tello.send_rc_control(0, 0, 0, 0)
                time.sleep(0.1)
                print("DEBUG: Enviando comando land() por Ctrl+C")
                node.tello.land()
            except Exception as land_err_ctrlc:
                print(f"Error durante el aterrizaje por Ctrl+C: {land_err_ctrlc}")
        # El flujo continuará al finally para la limpieza general
    except SystemExit as e:
        print(f"SystemExit detectado (código: {e.code}), probablemente cierre iniciado desde el nodo.")
        exit_code = e.code if isinstance(e.code, int) else 1
    except RuntimeError as e:
         print(f"Error de ejecución durante la inicialización: {e}")
         traceback.print_exc()
         exit_code = 1
    except Exception as e:
        print(f"Excepción no controlada en main: {type(e).__name__}: {e}")
        traceback.print_exc()
        if node: node.get_logger().fatal("Excepción fatal en main, forzando parada.")
        # Intentar aterrizar también en caso de excepción general
        if node and node.tello_connected and node.tello.is_flying:
             try:
                 print("DEBUG: Enviando RC -> 0,0,0,0 (Parada por Excepción)")
                 node.tello.send_rc_control(0, 0, 0, 0)
                 time.sleep(0.1)
                 print("DEBUG: Enviando comando land() por Excepción")
                 node.tello.land()
             except Exception as land_err_exc:
                 print(f"Error durante el aterrizaje por Excepción: {land_err_exc}")
        exit_code = 1
    finally:
        print("--- Bloque finally en main ---")
        if node:
            print("Llamando a node.cleanup()...")
            node.cleanup()
            if node.is_valid():
                 print("Llamando a node.destroy_node()...")
                 node.destroy_node()
            else:
                 print("El nodo ya no era válido.")
        else:
            print("El nodo no fue creado o ya fue limpiado.")

        if rclpy.ok():
            print("Llamando a rclpy.shutdown()...")
            rclpy.shutdown()
        else:
            print("rclpy ya estaba cerrado.")

        print(f"Programa ROS 2 Tello finalizado (Código de salida: {exit_code}).")

if __name__ == '__main__':
    main()