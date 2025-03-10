"""Primero arrancar QGroundControl y 3 instancias de drones en PX4; 
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="1,1 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="2,1 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 2
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="1,1 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 3

Comprobar con mavlink status los puertos, pero teoricamente se conectaras en 14541,14542,14543
Arrancar 3 mavsdk_server; 
/home/alvaro/.local/lib/python3.10/site-packages/mavsdk/bin/mavsdk_server -p 50040 udp://:14541
/home/alvaro/.local/lib/python3.10/site-packages/mavsdk/bin/mavsdk_server -p 50041 udp://:14542
/home/alvaro/.local/lib/python3.10/site-packages/mavsdk/bin/mavsdk_server -p 50042 udp://:14543

Finalmente ejecutar este archivo; python3 3drones.py """

import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)

async def run_drone(drone, port, altitude, move_x, move_y):
    """ Conecta, arma, despega, mueve en metros y aterriza un dron """
    await drone.connect()

    print(f"Dron en puerto {port} conectando...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Dron en puerto {port} conectado")
            break

    await drone.action.arm()
    print(f"Dron en puerto {port} armado")

    await drone.action.takeoff()
    print(f"Dron en puerto {port} despegando...")

    await asyncio.sleep(5)  # Esperar a que suba

    # Habilitar modo Offboard
    try:
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -altitude, 0.0))
        await drone.offboard.start()
        print(f"Dron en puerto {port} en modo Offboard")
    except OffboardError as e:
        print(f"Error en Offboard para dron {port}: {e}")
        return

    # Moverse en metros respecto a la posición inicial
    print(f"Dron en puerto {port} moviéndose a ({move_x}, {move_y}) metros")
    await drone.offboard.set_position_ned(PositionNedYaw(move_x, move_y, -altitude, 0.0))
    await asyncio.sleep(5)  # Esperar a que se mueva

    # Volver a la posición inicial
    print(f"Dron en puerto {port} regresando al punto de inicio")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -altitude, 0.0))
    await asyncio.sleep(5)

    await drone.offboard.stop()
    print(f"Dron en puerto {port} saliendo de Offboard")

    await drone.action.land()
    print(f"Dron en puerto {port} aterrizando...")

async def main():
    drone1 = System(mavsdk_server_address="localhost", port=50040)
    drone2 = System(mavsdk_server_address="localhost", port=50041)
    drone3 = System(mavsdk_server_address="localhost", port=50042)

    await asyncio.gather(
        run_drone(drone1, 50040, 10, 5, 5),  # Se mueve 5m en X y 5m en Y
        run_drone(drone2, 50041, 12, -5, 5), # Se mueve -5m en X y 5m en Y
        run_drone(drone3, 50042, 15, 5, -5)  # Se mueve 5m en X y -5m en Y
    )

if __name__ == "__main__":
    asyncio.run(main())

