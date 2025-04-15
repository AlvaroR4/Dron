import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import socket
import threading 

from mavsdk import System


SERVER_IP = "127.0.0.1"
SERVER_PORT = 65432


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/world/puertas_rojas/model/x500_mono_cam_0/link/camera_link/sensor/imager/image',
            self.listener_callback,
            10)
        self.br = CvBridge()

        # --- Variables para compartir la última posición ---
        self.latest_offset_x = 0
        self.latest_offset_y = 0
        self.offset_lock = threading.Lock() # Lock para acceso seguro entre hilos
        self._stop_event = threading.Event() # Evento para detener el hilo limpiamente

        # --- Iniciar el hilo para enviar posición en segundo plano ---
        self.sender_thread = threading.Thread(target=self.background_sender, daemon=True)
        self.sender_thread.start()
        self.get_logger().info('Background sender thread started.')

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        current_frame = self.br.imgmsg_to_cv2(data)
        imgRGB = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
        height, width, _ = imgRGB.shape
        center_x = width // 2
        center_y = height // 2
        lower_red = np.array([0, 0, 0]) 
        upper_red = np.array([255, 0, 0])

        mask = cv2.inRange(current_frame, lower_red, upper_red)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        found_target = False
        for c in contours:
            M = cv2.moments(c)
            if M['m00'] > 100: 
                found_target = True
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                cv2.circle(imgRGB, (cX, cY), 7, (255, 255, 255), -1)
                cv2.drawContours(imgRGB, [c], -1, (0, 255, 0), 3)

                offset_x = cX - center_x
                offset_y = cY -center_y

                with self.offset_lock:
                    self.latest_offset_x = offset_x
                    self.latest_offset_y = offset_y
                self.get_logger().debug(f'Target found. Updated offset: {offset_x}, {offset_y}') 

                cv2.putText(imgRGB, f"x_off: {offset_x}, y_off: {offset_y}", (cX - 20, cY - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                break 

        if not found_target:
             self.get_logger().debug('No target found in this frame.')


        cv2.imshow("Image processed", imgRGB)
        #cv2.imshow("Mask", mask) 
        cv2.waitKey(1)

    def background_sender(self):
        """Función ejecutada por el hilo para enviar la posición UDP periódicamente."""
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            self.get_logger().info('Background sender socket created.')
            while not self._stop_event.is_set():
                with self.offset_lock:
                    current_x = self.latest_offset_x
                    current_y = self.latest_offset_y

                message = f"{current_x},{current_y}"

                try:
                    sock.sendto(message.encode(), (SERVER_IP, SERVER_PORT))
                    self.get_logger().debug(f"Background position sent: {message}")
                except Exception as e:
                    self.get_logger().error(f"Error sending UDP: {e}")

                self._stop_event.wait(2.0)

        self.get_logger().info('Background sender thread finished.')

    def destroy_node(self):
        """Limpia recursos al destruir el nodo."""
        self.get_logger().info('Stopping background sender thread...')
        self._stop_event.set() # Señaliza al hilo que debe detenerse
        self.sender_thread.join(timeout=3.0) # Espera a que el hilo termine (con timeout)
        if self.sender_thread.is_alive():
            self.get_logger().warn('Background sender thread did not stop gracefully.')
        cv2.destroyAllWindows() # Cierra ventanas de OpenCV
        super().destroy_node() # Llama al método de la clase padre


def main(args=None):

    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()

    try:
        print("--> Entrando en rclpy.spin(). Presiona Ctrl+C para salir.")
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        print("--> Ctrl+C detectado. Saliendo...")
    except Exception as e:
        print(f"Error durante rclpy.spin(): {e}")
    finally:
        print("--> Realizando limpieza...")
        image_subscriber.destroy_node()
        rclpy.shutdown()
        print("--> Programa finalizado.")


if __name__ == '__main__':
    main()
