import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import socket



import asyncio

from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed)

drone = System()  # Objeto global del dron

# Configuración del socket (IP del servidor y puerto)
SERVER_IP = "127.0.0.1"  # Dirección local (localhost)
SERVER_PORT = 65432       # Puerto donde enviará los datos


async def send_position(x,y):
    """ Envía la posición del rover cada 2 segundos a un servidor UDP """
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        while True:
            current_x = x
            current_y = y
            message = f"{current_x},{current_y}"
                
            # Enviar mensaje al servidor
            sock.sendto(message.encode(), (SERVER_IP, SERVER_PORT))
            print(f"📤 Posición enviada: {message}")

            await asyncio.sleep(1)  # Enviar posición cada 2 segundos
            break  # Salir del `async for` después de la primera lectura

async def iniciar_drone():
    global drone
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break

    print("Waiting for drone to be armable...")
    async for health in drone.telemetry.health():
        print(f"-- Health: {health}")
        if health.is_global_position_ok and health.is_home_position_ok and health.is_armable:
            break
        await asyncio.sleep(1)

    async for mode in drone.telemetry.flight_mode():
        print(f"-- Flight Mode: {mode}")
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

    print("-- Despego")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, -2.0, 0.0))
    await asyncio.sleep(5)

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
                send_position(offset_x, offset_y)
                #moverDron(offset_x, offset_y)
                cv2.putText(imgRGB, f"x_off: {offset_x}, y_off: {offset_y}", (cX - 20, cY - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.imshow("Image processed", imgRGB)

        cv2.waitKey(1)
"""
async def moverDron(offset_x, offset_y):
    tolerancia = 1  # píxeles

    if abs(offset_x) < tolerancia and abs(offset_y) < tolerancia:
        print("Dron centrado con la puerta")
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        return

    speed = 1.5
    dx = offset_x
    dy = offset_y

    dist = (dx**2 + dy**2) ** 0.5
    velocidad_x = 0
    velocidad_y = dx / dist * speed
    velocidad_z = dy / dist * speed
    yaw = 0.0 

    print(f"Moviendo dron: x={velocidad_x}, y={velocidad_y}, z={velocidad_z}, yaw={yaw}")

    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(velocidad_x, velocidad_y, velocidad_z, yaw))
"""

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
    print("-- Comienzo fase inicio dron")
    asyncio.run(iniciar_drone())
    print("-- Fin fase inicio dron")
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
