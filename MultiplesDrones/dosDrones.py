#!/usr/bin/env python3

import asyncio
from mavsdk import System

async def run():
    # Conectar al primer dron (por defecto en puerto 14540)
    drone1 = System()
    await drone1.connect(system_address="udp://:14540")
    
    # Conectar al segundo dron (por ejemplo, en puerto 14541)
    drone2 = System()
    await drone2.connect(system_address="udp://:14541")
    
    # Esperar a que se conecte el primer dron
    print("Waiting for drone1 to connect...")
    async for state in drone1.core.connection_state():
        if state.is_connected:
            print("-- Drone1 connected!")
            break

    # Esperar a que se conecte el segundo dron
    print("Waiting for drone2 to connect...")
    async for state in drone2.core.connection_state():
        if state.is_connected:
            print("-- Drone2 connected!")
            break

    # Ejemplo: armar y despegar ambos drones
    print("-- Arming drone1")
    await drone1.action.arm()
    print("-- Arming drone2")
    await drone2.action.arm()

    print("-- Drone1 Taking off")
    await drone1.action.takeoff()
    print("-- Drone2 Taking off")
    await drone2.action.takeoff()

    await asyncio.sleep(10)

    print("-- Landing drone1")
    await drone1.action.land()
    print("-- Landing drone2")
    await drone2.action.land()

if __name__ == "__main__":
    asyncio.run(run())

