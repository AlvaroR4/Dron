#!/usr/bin/env python3


import asyncio
import socket
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed)

SERVER_IP = "127.0.0.1"
SERVER_PORT = 65432

""""
async def recibir_posiciones(sock):
        nonlocal leader_start_x, leader_start_y
        loop = asyncio.get_event_loop()
        while True:
            try:
                data, addr = await loop.run_in_executor(None, sock.recvfrom, 1024)
                mensaje = data.decode()
                print(f"he recibido: {mensaje}")

                x_str, y_str = mensaje.split(",")
                leader_x, leader_y = float(x_str), float(y_str)

                if leader_start_x is None:
                    leader_start_x = leader_x
                    leader_start_y = leader_y
                    print(f"📍 Posición inicial del líder: ({leader_x:.2f}, {leader_y:.2f})")
                    continue

                dx = leader_x - leader_start_x
                dy = leader_y - leader_start_y
                last_position["x"] = follower_start_x + dx
                last_position["y"] = follower_start_y + dy

                print(f"diferencia x: {dx:.2f}, diferencia y: {dy:.2f} . Objetivo seguidor: ({last_position['x']:.2f}, {last_position['y']:.2f})")

            except BlockingIOError:
                await asyncio.sleep(0.1)
                continue
"""

async def run():
    """ Does Offboard control using velocity body coordinates. """

    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    # Crear socket y recibir primera posición del líder
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    sock.bind((SERVER_IP, SERVER_PORT))
    loop = asyncio.get_running_loop()

    print("-- Arming")
    await drone.action.arm()

    print("-- Setting initial setpoint")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: \
              {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    print("-- Subo")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, -2.0, 0.0))
    await asyncio.sleep(6)






    print("-- Fin movimientos")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(8)

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: \
              {error._result.result}")
        
    print(f"---Dron aterrizando")
    await drone.action.land()


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
