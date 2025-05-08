# midas_gates_publisher.py
# Se suscribe a un topic de imagen ROS 2, aplica MiDaS para estimar profundidad,
# detecta "puertas" basándose en la profundidad, envía datos UDP
# y PUBLICA una imagen de visualización en un topic ROS 2.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy # Para QoS
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import torch
import numpy as np
import socket
import traceback # Para errores detallados

# --- Configuración ---
CAMARA_TOPIC_ENTRADA = '/world/puertas3/model/x500_mono_cam_0/link/camera_link/sensor/imager/image'
DISPLAY_TOPIC_SALIDA = '/midas_gates/display' # Topic para la imagen de visualización

SERVER_IP = "127.0.0.1"
SERVER_PORT = 65432

MODEL_TYPE = "MiDaS_small"
# Umbrales y parámetros para detección de puertas por profundidad (AJUSTA ESTOS)
UMBRAL_PROFUNDIDAD_PUERTA_LEJANA = 5.0  # Metros: Considerar regiones MÁS LEJANAS que esto como posibles puertas (ej. el hueco)
UMBRAL_PROFUNDIDAD_PUERTA_CERCANA = 0.3 # Metros: Ignorar regiones DEMASIADO CERCANAS (paredes, suelo inmediato)
MIN_AREA_PUERTA_PIXELS = 1500      # Área mínima en píxeles para considerar una región como puerta (AJUSTA)
# FACTOR_EXPANSION_ROI = 1.2       # No se usa en esta lógica, se puede quitar

class MidasGatesPublisher(Node):
    def __init__(self):
        # Cambiamos nombre para reflejar que publica
        super().__init__('midas_gates_publisher')
        self.get_logger().info(f"Iniciando Nodo Publicador de Puertas MiDaS.")
        self.get_logger().info(f"Suscribiendo a: {CAMARA_TOPIC_ENTRADA}")
        self.get_logger().info(f"Publicando visualización en: {DISPLAY_TOPIC_SALIDA}")
        self.get_logger().info(f"Enviando datos de control a UDP {SERVER_IP}:{SERVER_PORT}")

        # Suscripción a la imagen de entrada
        self.subscription = self.create_subscription(
            Image,
            CAMARA_TOPIC_ENTRADA,
            self.listener_callback,
            10) # QoS profile depth
        self.subscription
        self.bridge = CvBridge()

        # Socket UDP para enviar datos de control
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_address = (SERVER_IP, SERVER_PORT)

        # Cargar modelo MiDaS (igual que antes)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"Cargando modelo MiDaS ({MODEL_TYPE}) en {self.device}...")
        try:
            self.midas = torch.hub.load("intel-isl/MiDaS", MODEL_TYPE, trust_repo=True)
            self.midas.to(self.device)
            self.midas.eval()
            midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms", trust_repo=True)
            self.transform = midas_transforms.small_transform if MODEL_TYPE == "MiDaS_small" else midas_transforms.dpt_transform
            self.get_logger().info("Modelo MiDaS y transformaciones cargadas.")
        except Exception as e:
            self.get_logger().fatal(f"Error cargando modelo MiDaS: {e}. Asegúrate de tener conexión y 'timm' instalado.")
            traceback.print_exc()
            rclpy.shutdown()
            raise SystemExit(f"Fallo al cargar modelo MiDaS: {e}")

        # --- Crear el Publicador para la Imagen de Visualización ---
        qos_profile_display = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.display_publisher_ = self.create_publisher(
            Image,
            DISPLAY_TOPIC_SALIDA,
            qos_profile_display
        )
        self.get_logger().info(f"Publicador de visualización creado en '{DISPLAY_TOPIC_SALIDA}'.")
        
        # NO HAY LLAMADAS A cv2.namedWindow AQUÍ
        self.get_logger().info("Inicialización completa. Esperando mensajes ROS...")


    def listener_callback(self, msg):
        self.get_logger().info(f'Callback: Recibiendo frame {msg.header.stamp.sec}', throttle_duration_sec=1.0)
        try:
            # Convertir a OpenCV BGR y RGB
            cv_image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image_rgb = cv2.cvtColor(cv_image_bgr, cv2.COLOR_BGR2RGB)
            altura, anchura, _ = cv_image_bgr.shape # Guardar dimensiones
        except CvBridgeError as e:
            self.get_logger().error(f"Error CvBridge: {e}")
            return
        except Exception as e:
             self.get_logger().error(f"Error convirtiendo imagen: {e}")
             return

        try:
            # --- Procesamiento MiDaS ---
            input_batch = self.transform(cv_image_rgb).to(self.device)
            with torch.no_grad():
                prediction = self.midas(input_batch)
                prediction = torch.nn.functional.interpolate(
                    prediction.unsqueeze(1),
                    size=(altura, anchura), # Usar dimensiones reales
                    mode="bicubic",
                    align_corners=False,
                ).squeeze()
            depth_map = prediction.cpu().numpy() # Mapa de profundidad inverso (valores bajos=lejos)

            # Invertir mapa para que valores altos=lejos (opcional pero a veces más intuitivo)
            # O escalar la predicción directamente. Vamos a usarlo como está y ajustar umbrales.
            # Nota: MiDaS da profundidad inversa o disparidad. Valores más altos están más cerca.
            # Reajustamos la lógica de umbrales pensando en esto.
            # Umbral cercano: Ignorar píxeles con valor de profundidad ALTO (muy cerca)
            # Umbral lejano: Considerar píxeles con valor de profundidad BAJO (lejos) como puerta
            # Es más fácil trabajar con distancia métrica si el modelo la da, pero MiDaS_small da relativa.
            # Vamos a normalizar y umbralizar sobre eso, asumiendo que lo lejano (puerta)
            # tendrá valores normalizados bajos y lo cercano valores altos.

            depth_norm_vis = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            depth_colored_vis = cv2.cvtColor(depth_norm_vis, cv2.COLOR_GRAY2BGR) # Para dibujar encima

            # --- Detección de "Puertas" por Umbral Inverso de Profundidad (Altos = Cerca) ---
            # Queremos el hueco (lejos = valores bajos de depth_map) pero no lo más cercano.
            # Necesitamos ajustar los umbrales pensando en el rango de salida de MiDaS
            # Es más fácil normalizar primero y luego umbralizar en 0-255
            
            # Ejemplo: buscar regiones con valor normalizado BAJO (lejos) pero no extremadamente bajo
            UMBRAL_LEJANO_NORM = 50  # Considerar píxeles más oscuros que esto como posible hueco (lejos)
            UMBRAL_CERCANO_NORM = 150 # Ignorar píxeles más brillantes que esto (cerca) - Ajustar

            # mascara_hueco = (depth_norm_vis < UMBRAL_LEJANO_NORM)
            # mascara_no_tan_cerca = (depth_norm_vis < UMBRAL_CERCANO_NORM)
            # mascara_puertas = mascara_hueco & mascara_no_tan_cerca # Quedarnos con lo lejano pero no lo más cercano
            
            # Alternativa más simple: Buscar solo las regiones "lejanas" (valores bajos en depth_map original)
            # Obtener un umbral basado en percentiles puede ser más robusto
            percentil_lejano = np.percentile(depth_map, 15) # Considerar el 15% más lejano como puerta
            mascara_puertas = (depth_map < percentil_lejano).astype(np.uint8) * 255

            # Opcional: Operaciones morfológicas para limpiar la máscara
            kernel = np.ones((7,7),np.uint8)
            mascara_puertas = cv2.morphologyEx(mascara_puertas, cv2.MORPH_OPEN, kernel, iterations=1)
            mascara_puertas = cv2.morphologyEx(mascara_puertas, cv2.MORPH_CLOSE, kernel, iterations=2)

            contornos_puertas, _ = cv2.findContours(mascara_puertas, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            puertas_detectadas = []
            for contorno in contornos_puertas:
                area = cv2.contourArea(contorno)
                if area > MIN_AREA_PUERTA_PIXELS:
                    x, y, w, h = cv2.boundingRect(contorno)
                    centro_x = x + w // 2
                    centro_y = y + h // 2
                    
                    # Usar el valor MEDIO de profundidad en la región detectada como "distancia"
                    # (Recordar que es inversa o relativa, no metros directos con MiDaS Small)
                    distancia_relativa = np.mean(depth_map[y:y+h, x:x+w]) 
                    
                    puertas_detectadas.append({
                        'bbox': (x, y, w, h),
                        'centro': (centro_x, centro_y),
                        'distancia': distancia_relativa, # Usamos el valor relativo
                        'area': area
                    })

            # --- Selección y Envío UDP ---
            puerta_mas_cercana = None
            offset_x_puerta, offset_y_puerta = 0.0, 0.0
            # La "distancia" que enviamos es la relativa/inversa de MiDaS (menor = más lejos)
            distancia_a_enviar = float('inf') # Valor alto indica "no detectado" o muy cerca
            num_puertas_detectadas = len(puertas_detectadas)

            if num_puertas_detectadas > 0:
                # Ordenar por 'distancia' (valor bajo es lejos, así que buscamos el mínimo)
                puertas_detectadas.sort(key=lambda p: p['distancia'])
                puerta_mas_cercana = puertas_detectadas[0] 

                centro_x_imagen = anchura // 2
                centro_y_imagen = altura // 2
                offset_x_puerta = float(puerta_mas_cercana['centro'][0] - centro_x_imagen)
                offset_y_puerta = float(puerta_mas_cercana['centro'][1] - centro_y_imagen)
                distancia_a_enviar = puerta_mas_cercana['distancia'] # Enviar valor relativo MiDaS

            # Formatear y enviar mensaje UDP
            # ¡OJO! El script que recibe debe interpretar 'distancia_a_enviar' correctamente
            # (no son metros, valores bajos significan lejos).
            message = f"{offset_x_puerta},{offset_y_puerta},{distancia_a_enviar:.4f},{num_puertas_detectadas}"
            try:
                self.sock.sendto(message.encode('utf-8'), self.server_address)
            except Exception as e_udp:
                self.get_logger().error(f"Error enviando UDP: {e_udp}", throttle_duration_sec=5.0)

            # --- Preparar Imagen de Visualización y Publicar ---
            # Dibujar sobre la imagen de profundidad coloreada
            for i, puerta in enumerate(puertas_detectadas):
                x, y, w, h = puerta['bbox']
                if puerta == puerta_mas_cercana:
                    color = (255, 0, 0) # Azul para la más cercana (la más lejana en profundidad real)
                    grosor = 2
                    info_text = f"OBJ D:{puerta['distancia']:.2f} A:{puerta['area']:.0f}"
                    cv2.putText(depth_colored_vis, info_text, (x, y - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                else:
                    color = (0, 165, 255) # Naranja para otras
                    grosor = 1
                
                cv2.rectangle(depth_colored_vis, (x, y), (x + w, y + h), color, grosor)
                cv2.circle(depth_colored_vis, puerta['centro'], 3, color, -1)

            # Publicar la imagen de visualización
            try:
                vis_msg_out = self.bridge.cv2_to_imgmsg(depth_colored_vis, encoding="bgr8")
                vis_msg_out.header.stamp = msg.header.stamp
                self.display_publisher_.publish(vis_msg_out)
            except CvBridgeError as e_pub:
                 self.get_logger().error(f"Error CvBridge al publicar visualización: {e_pub}")
            except Exception as e_pub_gen:
                 self.get_logger().error(f"Error al publicar visualización: {e_pub_gen}")

        except Exception as e:
            self.get_logger().error(f"Error general en callback: {e}")
            traceback.print_exc()

        # --- NO HAY cv2.imshow / cv2.waitKey ---


    def destroy_node(self):
        self.get_logger().info('Cerrando nodo publicador de puertas MiDaS...')
        if hasattr(self, 'sock') and self.sock:
            self.sock.close()
            self.get_logger().info('Socket UDP cerrado.')
        # --- NO HAY cv2.destroyAllWindows ---
        super().destroy_node()

# --- Main (similar al anterior) ---
def main(args=None):
    rclpy.init(args=args)
    midas_gates_publisher_node = None
    try:
        midas_gates_publisher_node = MidasGatesPublisher()
        if midas_gates_publisher_node and rclpy.ok():
             midas_gates_publisher_node.get_logger().info("Iniciando bucle rclpy.spin()...")
             rclpy.spin(midas_gates_publisher_node)
        else:
             if not midas_gates_publisher_node: print("Error: No se pudo crear el nodo.")
    except KeyboardInterrupt:
        print("Ctrl+C detectado, cerrando nodo...")
    except Exception as e:
        print(f"Error inesperado en main: {e}")
        traceback.print_exc()
    finally:
        print("Bloque finally en main...")
        if midas_gates_publisher_node:
            try: midas_gates_publisher_node.destroy_node()
            except Exception as destroy_e: print(f"Error durante destroy_node: {destroy_e}")
        if rclpy.ok():
            print("Cerrando rclpy...")
            rclpy.shutdown()
        print("Programa finalizado.")

if __name__ == '__main__':
    main()