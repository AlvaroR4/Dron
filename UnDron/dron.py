#!/usr/bin/env python3


#Para ejecutarlo basta con crear un dron con make px4_sitl gz_x500 y lanzar un QGroundControl
#No hace falta mavsdk_server porque como solo es un dron se conectará al 14540 de px4

import asyncio  # Módulo para trabajar con programación asíncrona (await, async)
from mavsdk import System  # Importa la clase principal del dron
from mavsdk.offboard import (PositionNedYaw, VelocityNedYaw, OffboardError)
# Importa clases necesarias para el control en modo Offboard

async def run():
    drone = System()  # Crea una instancia del dron
    await drone.connect(system_address="udp://:14540")  # Conecta al dron por UDP (usualmente PX4 en SITL)

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():  # Espera hasta que el dron esté conectado
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():  # Espera a que haya posición global y posición inicial establecida
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()  # Arma el dron (enciende los motores)

    print("-- Setting initial setpoint")
    # Define un primer objetivo de posición (0,0,0) con yaw 0 para iniciar el modo offboard
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()  # Intenta activar el modo offboard (control manual desde el código)
    except OffboardError as error:  # Si falla, desarma el dron y termina
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    # Función para imprimir continuamente la velocidad en el eje Z (down)
    async def print_z_velocity(drone):
        async for odom in drone.telemetry.position_velocity_ned():
            print(f"{odom.velocity.north_m_s} {odom.velocity.down_m_s}")  # Muestra velocidades Norte y Z

    # Ejecuta esa función en segundo plano
    asyncio.ensure_future(print_z_velocity(drone))

    print("-- Go 0m North, 0m East, -10m Down within local coordinate system")
    # Mueve el dron a 10 metros hacia abajo con una velocidad descendente de 1 m/s
    await drone.offboard.set_position_velocity_ned(
        PositionNedYaw(0.0, 0.0, -10.0, 0.0),  # Posición deseada
        VelocityNedYaw(0.0, 0.0, -1.0, 0.0))   # Velocidad deseada
    await asyncio.sleep(10)  # Espera 10 segundos

    print("-- Go 10m North, 0m East, 0m Down within local coordinate system")
    # Luego se mueve hacia el norte (10 metros) a una velocidad de 1 m/s
    await drone.offboard.set_position_velocity_ned(
        PositionNedYaw(50.0, 0.0, -10.0, 0.0),  # Nueva posición objetivo
        VelocityNedYaw(1.0, 0.0, 0.0, 0.0))     # Velocidad hacia el norte
    await asyncio.sleep(20)  # Espera 20 segundos


    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()  # Intenta salir del modo offboard
    except OffboardError as error:  # Muestra error si no puede detenerlo
        print(f"Stopping offboard mode failed with error code: {error._result.result}")

    # Instrucción para aterrizar el dron
    await drone.action.land()


if __name__ == "__main__":
    # Ejecuta el bucle principal asyncio
    asyncio.run(run())
