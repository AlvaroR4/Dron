import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import socket


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
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_address = (SERVER_IP, SERVER_PORT)
        self.get_logger().info(f"UDP Socket created for sending to {SERVER_IP}:{SERVER_PORT}")


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
        offset_x = 0
        offset_y = 0

        for c in contours:
            M = cv2.moments(c)
            if M['m00'] > 100:
                found_target = True
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                cv2.circle(imgRGB, (cX, cY), 7, (255, 255, 255), -1)
                cv2.drawContours(imgRGB, [c], -1, (0, 255, 0), 3)

                offset_x = cX - center_x
                offset_y = cY - center_y

                self.get_logger().debug(f'Target found. Calculated offset: {offset_x}, {offset_y}')

                cv2.putText(imgRGB, f"x_off: {offset_x}, y_off: {offset_y}", (cX - 20, cY - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                break

        if not found_target:
             self.get_logger().debug('No target found in this frame. Sending offset 0,0.')

        message = f"{offset_x},{offset_y}"
        try:
            self.sock.sendto(message.encode('utf-8'), self.server_address)
        except Exception as e:
            self.get_logger().error(f"Error sending UDP in callback: {e}")


        cv2.imshow("Image processed", imgRGB)
        cv2.waitKey(1)


    def destroy_node(self):
        self.get_logger().info('Cleaning up node...')
        if hasattr(self, 'sock') and self.sock:
             try:
                 self.sock.close()
                 self.get_logger().info('UDP Socket closed.')
             except Exception as e:
                  self.get_logger().error(f"Error closing UDP socket: {e}")
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)
    image_subscriber = None
    try:
        image_subscriber = ImageSubscriber()
        print("--> Entrando en rclpy.spin(). Presiona Ctrl+C para salir.")
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        print("--> Ctrl+C detectado. Saliendo...")
    except Exception as e:
        if image_subscriber:
             image_subscriber.get_logger().fatal(f"Error crítico durante rclpy.spin(): {e}", exc_info=True)
        else:
             print(f"Error crítico antes de completar inicialización: {e}")

    finally:
        print("--> Realizando limpieza...")
        if image_subscriber:
             image_subscriber.destroy_node()
        if rclpy.ok():
             rclpy.shutdown()
        print("--> Programa finalizado.")


if __name__ == '__main__':
    main()