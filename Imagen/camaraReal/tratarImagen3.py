import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import socket
import threading
import time 


SERVER_IP = "127.0.0.1"
SERVER_PORT = 65432


class ProcesadorImagen(Node): 
    def __init__(self):
        super().__init__('procesador_imagen_real')

        self.declare_parameter('topic_entrada_imagen', '/camara_real/image_raw')
        topic_entrada = self.get_parameter('topic_entrada_imagen').get_parameter_value().string_value
        self.get_logger().info(f"Suscribiéndose al topic de imagen: '{topic_entrada}'")

        self.subscription = self.create_subscription(
            Image,
            topic_entrada, 
            self.listener_callback,
            10) 
        self.br = CvBridge()

        self.latest_offset_x = 0
        self.latest_offset_y = 0
        self.offset_lock = threading.Lock() 
        self._stop_event = threading.Event()

        self.sender_thread = threading.Thread(target=self.background_sender, daemon=True)
        self.sender_thread.start()
        self.get_logger().info('Hilo de envío UDP en segundo plano iniciado.')

    def listener_callback(self, data):
        self.get_logger().debug('Procesando frame de vídeo recibido...')
        try:
            current_frame_bgr = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except Exception as e:
             self.get_logger().error(f"Fallo en conversión de CvBridge: {e}")
             return

        img_display = current_frame_bgr.copy() 
        height, width, _ = current_frame_bgr.shape
        center_x = width // 2
        center_y = height // 2

  
        lower_color = np.array([0, 0, 0]) 
        upper_color = np.array([255, 0, 0]) 

        mask = cv2.inRange(current_frame_bgr, lower_color, upper_color)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        found_target = False
        current_offset_x = 0 
        current_offset_y = 0

        if contours: 
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M['m00'] > 100:
                found_target = True
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                cv2.circle(img_display, (cX, cY), 7, (0, 255, 0), -1) 
                cv2.drawContours(img_display, [c], -1, (0, 255, 0), 3)

                current_offset_x = cX - center_x
                current_offset_y = cY - center_y

                cv2.putText(img_display, f"x_off: {current_offset_x}, y_off: {current_offset_y}", (cX - 50, cY - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2) 

        with self.offset_lock:
            self.latest_offset_x = current_offset_x
            self.latest_offset_y = current_offset_y
            if found_target:
                self.get_logger().debug(f'Objetivo encontrado. Offset actualizado: {current_offset_x}, {current_offset_y}')


        cv2.circle(img_display, (center_x, center_y), 5, (0, 0, 255), -1) 
        cv2.imshow("Imagen Procesada (Real)", img_display)
        cv2.waitKey(1) 

    def background_sender(self):
        """Función ejecutada por el hilo para enviar la posición UDP periódicamente."""
        time.sleep(2.0)
        self.get_logger().info(f'Empezando envío UDP a {SERVER_IP}:{SERVER_PORT}')
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            self.get_logger().info('Socket UDP del hilo de envío creado.')
            while not self._stop_event.is_set():
                with self.offset_lock:
                    current_x = self.latest_offset_x
                    current_y = self.latest_offset_y

                message = f"{current_x},{current_y}"

                try:
                    sock.sendto(message.encode('utf-8'), (SERVER_IP, SERVER_PORT))
                    self.get_logger().debug(f"Posición enviada por UDP: {message}")
                except socket.error as e:
                    self.get_logger().error(f"Error de socket al enviar UDP: {e}")
                except Exception as e:
                    self.get_logger().error(f"Error inesperado al enviar UDP: {e}")

                self._stop_event.wait(1.0)

        self.get_logger().info('Hilo de envío UDP finalizado.')

    def destroy_node(self):
        """Limpia recursos al destruir el nodo."""
        self.get_logger().info('Iniciando cierre del nodo procesador...')
        self._stop_event.set() 
        if hasattr(self, 'sender_thread') and self.sender_thread.is_alive():
             self.get_logger().info('Esperando a que termine el hilo de envío UDP...')
             self.sender_thread.join(timeout=2.0)
             if self.sender_thread.is_alive():
                  self.get_logger().warn('El hilo de envío UDP no terminó correctamente.')
             else:
                  self.get_logger().info('Hilo de envío UDP terminado.')
        else:
             self.get_logger().info('El hilo de envío UDP no estaba activo o no se inició.')

        cv2.destroyAllWindows()
        self.get_logger().info('Ventanas de OpenCV cerradas.')
        super().destroy_node() 
        self.get_logger().info('Nodo procesador destruido.')


def main(args=None):
    rclpy.init(args=args)
    procesador = None 
    try:
        procesador = ProcesadorImagen()
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