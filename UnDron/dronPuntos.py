#!/usr/bin/env python3
import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Esperando a que el dron se conecte...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Dron conectado!")
            break

    print("Esperando a que se tenga una estimación global de posición...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Estimación global OK")
            break

    print("-- Armando el dron")
    await drone.action.arm()

    print("-- Despegando")
    await drone.action.takeoff()
    await asyncio.sleep(5)  # Espera a que se estabilice el despegue

    # Configuración del modo offboard:
    # Se utiliza la posición NED, donde:
    #   - Primer parámetro: desplazamiento al norte (en metros)
    #   - Segundo: desplazamiento al este (en metros)
    #   - Tercero: desplazamiento down (negativo para mantener altura)
    #   - Cuarto: yaw en grados (0 en este ejemplo)
    altura_objetivo = 2.0  # metros de altura (tomados como -2 en eje Down)
    posicion_inicial = PositionNedYaw(0.0, 0.0, -altura_objetivo, 0.0)
    
    # Envío de setpoint inicial antes de arrancar offboard
    try:
        await drone.offboard.set_position_ned(posicion_inicial)
        await drone.offboard.start()
        print("-- Offboard mode iniciado")
    except OffboardError as error:
        print(f"Error al iniciar offboard: {error._result.result}")
        await drone.action.disarm()
        return

    print("-- Ejecutando patrón cuadrado de 1 metro")
    # 1. Mover 1 m al norte
    await drone.offboard.set_position_ned(PositionNedYaw(100.0, 0.0, -altura_objetivo, 0.0))
    await asyncio.sleep(3)
    # 2. Mover 1 m al este
    await drone.offboard.set_position_ned(PositionNedYaw(100.0, 1.0, -altura_objetivo, 0.0))
    await asyncio.sleep(3)
    # 3. Mover 1 m al sur
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 100.0, -altura_objetivo, 0.0))
    await asyncio.sleep(3)
    # 4. Mover 1 m al oeste (regresar a la posición inicial)
    await drone.offboard.set_position_ned(posicion_inicial)
    await asyncio.sleep(3)

    try:
        await drone.offboard.stop()
        print("-- Offboard mode detenido")
    except OffboardError as error:
        print(f"Error al detener offboard: {error._result.result}")

    print("-- Aterrizando")
    await drone.action.land()
    await asyncio.sleep(10)
    print("Misión completada")

if __name__ == "__main__":
    asyncio.run(run())
