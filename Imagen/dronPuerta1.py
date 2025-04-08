import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


import asyncio

from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed)

drone = System()  # Objeto global del dron

async def iniciar_drone():
    global drone
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Setting initial setpoint")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard failed: {error._result.result}")
        await drone.action.disarm()

async def fin_drone():
    print("-- Stopping offboard")
    try:
        await drone.offboard.stop() 
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: {error._result.result}")

    await drone.action.land()

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
                offset_x = cX - center_x
                offset_y = cY - center_y
                moverDron(offset_x, offset_y)
                cv2.putText(imgRGB, f"x_off: {offset_x}, y_off: {offset_y}", (cX - 20, cY - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.imshow("Image processed", imgRGB)

        cv2.waitKey(1)

async def moverDron(offset_x, offset_y):
    tolerancia = 1  # píxeles

    if abs(offset_x) < tolerancia and abs(offset_y) < tolerancia:
        print("Dron centrado con la puerta")
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        return

    velocidad_x = 0
    velocidad_y = 0
    velocidad_z = 0
    yaw = 0.0 

    print(f"Moviendo dron: x={velocidad_x}, y={velocidad_y}, z={velocidad_z}, yaw={yaw}")

    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(velocidad_x, velocidad_y, velocidad_z, yaw))

"""
Ejemplo de función para mover; 

  void goTowards(float px, float py, float speed){
    float dx = px;
    float dy = py;
    float dist = sqrt(dx*dx + dy*dy);
    x += dx / dist * speed;
    y += dy / dist * speed;
  }
"""
def main(args=None):
    rclpy.init(args=args)
    asyncio.run(iniciar_drone())
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()