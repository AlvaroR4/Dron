import asyncio
import sys
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

async def main(num_drones):
    drones = []
    ports = [50040 + i for i in range(num_drones)]  # Generar puertos dinámicamente
    positions = [(10, 5), (12, -5), (15, 5)]  # Ejemplo de movimientos para los drones

    # Crear drones dinámicamente
    for i in range(num_drones):
        drone = System(mavsdk_server_address="localhost", port=ports[i])
        drones.append((drone, ports[i], positions[i][0], positions[i][1]))

    # Ejecutar todos los drones
    await asyncio.gather(
        *[run_drone(drone, port, 10, move_x, move_y) for drone, port, move_x, move_y in drones]
    )

if __name__ == "__main__":
    # Asegurarse de pasar el número de drones como argumento
    if len(sys.argv) < 2:
        print("Por favor, proporcione el número de drones como argumento.")
        sys.exit(1)
    
    num_drones = int(sys.argv[1])
    asyncio.run(main(num_drones))

