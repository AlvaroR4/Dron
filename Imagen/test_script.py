import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/world/default/model/x500_mono_cam_0/link/camera_link/sensor/imager/image',
            self.listener_callback,
            10)
        self.br = CvBridge()
    
    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        current_frame = self.br.imgmsg_to_cv2(data)
        imgRGB = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
        #cv2.imshow("camera", imgRGB)
        height, width, _ = imgRGB.shape
        center_x = width // 2
        center_y = height // 2
        lower_red = np.array([0, 0, 0])
        upper_red = np.array([255, 0, 0])
        
        mask = cv2.inRange(current_frame, lower_red, upper_red)
        #cv2.imshow("mask", mask)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, 
                           cv2.CHAIN_APPROX_SIMPLE)
        #print(contours)
        for c in contours:
            M = cv2.moments(c)
            if M['m00'] != 0:
                #cv2.imshow("mask", cnts)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                cv2.circle(imgRGB, (cX, cY), 7, (255, 255, 255), -1)
                cv2.drawContours(imgRGB, [c], -1, (0,255,0), 3)
                cv2.putText(imgRGB, f"x_off: {cX-center_x}, y_off: {cY-center_y}", (cX - 20, cY - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.imshow("Image processed", imgRGB)

        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()