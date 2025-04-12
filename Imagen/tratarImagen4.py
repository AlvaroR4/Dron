import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import socket
# Se elimina: import threading

# Ya no necesitas MAVSDK aquí si solo procesas imagen y envías UDP
# from mavsdk import System

# --- Configuración UDP ---
SERVER_IP = "127.0.0.1"
SERVER_PORT = 65432

class ProcesadorImagenSincrono(Node): # Nombre de clase actualizado
    def __init__(self):
        super().__init__('procesador_imagen_sincrono') # Nombre de nodo actualizado

        # --- Parámetro para el nombre del topic de entrada ---
        self.declare_parameter('topic_entrada_imagen', '/camara_real/image_raw') # Asegúrate que este es el topic correcto
        topic_entrada = self.get_parameter('topic_entrada_imagen').get_parameter_value().string_value
        self.get_logger().info(f"Suscribiéndose al topic de imagen: '{topic_entrada}'")

        # --- Suscripción ROS 2 (sin cambios) ---
        self.subscription = self.create_subscription(
            Image,
            topic_entrada,
            self.listener_callback,
            10)
        self.br = CvBridge()

        # --- Crear el socket UDP una vez al inicio ---
        self.server_address = (SERVER_IP, SERVER_PORT)
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.get_logger().info(f"Socket UDP creado para enviar a {SERVER_IP}:{SERVER_PORT}")
        except Exception as e:
            self.get_logger().fatal(f"No se pudo crear el socket UDP: {e}")
            # Considerar cerrar el nodo si el socket es esencial y falla
            rclpy.shutdown()

        # --- Ya NO se necesitan variables compartidas, lock, evento ni hilo ---
        # self.latest_offset_x = 0
        # self.latest_offset_y = 0
        # self.offset_lock = threading.Lock()
        # self._stop_event = threading.Event()
        # self.sender_thread = threading.Thread(target=self.background_sender, daemon=True)
        # self.sender_thread.start()
        # self.get_logger().info('Background sender thread started.')

    def listener_callback(self, data):
        """Procesa cada frame recibido y envía el offset UDP inmediatamente."""
        self.get_logger().debug('Procesando frame de vídeo recibido...')
        try:
            current_frame_bgr = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Fallo en conversión de CvBridge: {e}")
            return

        # --- Procesamiento de Imagen (lógica central sin cambios) ---
        img_display = current_frame_bgr.copy()
        height, width, _ = current_frame_bgr.shape
        center_x = width // 2
        center_y = height // 2

        # --- Definir rango de color (BGR) - AJUSTAR SEGÚN NECESIDAD ---
        lower_color = np.array([0, 0, 150])  # Ejemplo para rojo
        upper_color = np.array([80, 80, 255]) # Ejemplo para rojo

        mask = cv2.inRange(current_frame_bgr, lower_color, upper_color)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # --- Variables para el offset de ESTE frame ---
        current_offset_x = 0
        current_offset_y = 0
        found_target = False

        if contours:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M['m00'] > 100:
                found_target = True
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # Calcular offset para ESTE frame
                current_offset_x = cX - center_x
                current_offset_y = cY - center_y

                # Dibujar información (sin cambios)
                cv2.circle(img_display, (cX, cY), 7, (0, 255, 0), -1)
                cv2.drawContours(img_display, [c], -1, (0, 255, 0), 3)
                cv2.putText(img_display, f"x_off: {current_offset_x}, y_off: {current_offset_y}", (cX - 50, cY - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # --- Enviar datos UDP AHORA, después de procesar el frame ---
        # Se envía el offset calculado (o 0 si no se encontró objetivo)
        message = f"{current_offset_x},{current_offset_y}"
        try:
            self.sock.sendto(message.encode('utf-8'), self.server_address)
            self.get_logger().debug(f"Offset enviado por UDP: {message}")
        except Exception as e:
            self.get_logger().error(f"Error al enviar UDP: {e}")

        # --- Mostrar Imagen Procesada (sin cambios) ---
        cv2.circle(img_display, (center_x, center_y), 5, (0, 0, 255), -1) # Centro real
        cv2.imshow("Imagen Procesada (Síncrono)", img_display)
        cv2.waitKey(1)

    # --- Se elimina la función background_sender ---
    # def background_sender(self):
    #     ...

    def destroy_node(self):
        """Limpia recursos al destruir el nodo."""
        self.get_logger().info('Iniciando cierre del nodo procesador síncrono...')

        # --- Cerrar el socket UDP ---
        if hasattr(self, 'sock') and self.sock:
             try:
                  self.sock.close()
                  self.get_logger().info('Socket UDP cerrado.')
             except Exception as e:
                  self.get_logger().error(f"Error al cerrar el socket UDP: {e}")

        # --- Ya no hay hilo que detener ---
        # self._stop_event.set()
        # self.sender_thread.join(timeout=3.0)

        cv2.destroyAllWindows()
        self.get_logger().info('Ventanas de OpenCV cerradas.')
        super().destroy_node()
        self.get_logger().info('Nodo procesador síncrono destruido.')


def main(args=None):
    rclpy.init(args=args)
    procesador = None
    try:
        # Usar el nuevo nombre de la clase
        procesador = ProcesadorImagenSincrono()
        print("--> Entrando en el bucle principal (rclpy.spin). Presiona Ctrl+C para salir.")
        rclpy.spin(procesador)
    except KeyboardInterrupt:
        print("\n--> Ctrl+C detectado. Iniciando secuencia de apagado...")
    except Exception as e:
        if procesador:
            procesador.get_logger().fatal(f"Error crítico durante rclpy.spin(): {e}", exc_info=True)
        else:
            print(f"Error crítico antes de completar la inicialización del nodo: {e}")
    finally:
        print("--> Realizando limpieza final...")
        if procesador:
            procesador.destroy_node()
        else:
             print("--> El nodo no llegó a inicializarse completamente.")
        if rclpy.ok():
             rclpy.shutdown()
             print("--> rclpy cerrado.")
        print("--> Programa finalizado.")


if __name__ == '__main__':
    main()