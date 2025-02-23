#!/usr/bin/env python3

import asyncio
from mavsdk import System

async def run():
    """Controla el dron para despegar, moverse en cuadrado y aterrizar."""

    # Conectar al dron vía MAVSDK
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # Esperar conexión
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    # Esperar que el dron tenga una posición global válida
    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    # Obtener posición inicial para calcular desplazamientos
    print("Fetching initial position...")
    async for position in drone.telemetry.position():
        home_lat = position.latitude_deg
        home_lon = position.longitude_deg
        break

    # Armar y despegar
    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()
    await asyncio.sleep(5)  # Esperar a que el dron estabilice

    # Definir los waypoints en forma de cuadrado (ajustados desde la posición actual)
    square_size = 0.0001  # Aproximadamente 11 metros
    altitude = 10.0  # Altura de vuelo

    waypoints = [
        (home_lat, home_lon + square_size, altitude),  # Derecha
        (home_lat + square_size, home_lon + square_size, altitude),  # Arriba
        (home_lat + square_size, home_lon, altitude),  # Izquierda
        (home_lat, home_lon, altitude)  # Vuelta a inicio
    ]

    # Moverse a cada waypoint
    for lat, lon, alt in waypoints:
        print(f"-- Flying to waypoint: lat={lat}, lon={lon}, alt={alt}")
        await drone.action.goto_location(lat, lon, alt, 0)
        await asyncio.sleep(10)  # Esperar a que llegue

    # Aterrizar
    print("-- Landing")
    await drone.action.land()

    # Cancelar tareas activas
    status_text_task.cancel()

async def print_status_text(drone):
    """Monitorea los mensajes de estado del dron."""
    try:
        async for status_text in drone.telemetry.status_text():
            print(f"Status: {status_text.type}: {status_text.text}")
    except asyncio.CancelledError:
        return

if __name__ == "__main__":
    # Ejecutar el loop asyncio
    asyncio.run(run())

