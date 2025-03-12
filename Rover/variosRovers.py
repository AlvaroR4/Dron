import asyncio
import argparse
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

async def run_rover(rover, port, altitude, move_x, move_y):
    """Conecta, arma, despega, mueve en metros y aterriza un rover"""
    await rover.connect()

    print(f"Rover en puerto {port} conectando...")
    async for state in rover.core.connection_state():
        if state.is_connected:
            print(f"Rover en puerto {port} conectado")
            break

    await rover.action.arm()
    print(f"Rover en puerto {port} armado")

    await rover.action.takeoff()  # Para efectos de simulación se utiliza takeoff
    print(f"Rover en puerto {port} despegando...")

    await asyncio.sleep(5)  # Esperar a que suba

    # Habilitar modo Offboard
    try:
        await rover.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -altitude, 0.0))
        await rover.offboard.start()
        print(f"Rover en puerto {port} en modo Offboard")
    except OffboardError as e:
        print(f"Error en Offboard para rover {port}: {e}")
        return

    # Moverse en metros respecto a la posición inicial
    print(f"Rover en puerto {port} moviéndose a ({move_x}, {move_y}) metros")
    await rover.offboard.set_position_ned(PositionNedYaw(move_x, move_y, -altitude, 0.0))
    await asyncio.sleep(5)  # Esperar a que se mueva

    # Volver a la posición inicial
    print(f"Rover en puerto {port} regresando al punto de inicio")
    await rover.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -altitude, 0.0))
    await asyncio.sleep(5)

    await rover.offboard.stop()
    print(f"Rover en puerto {port} saliendo de Offboard")

    await rover.action.land()  # Lógica de aterrizaje para la simulación
    print(f"Rover en puerto {port} aterrizando...")

async def main(num_rovers):
    tasks = []
    for i in range(num_rovers):
        port = 50040 + i
        # Generamos parámetros de ejemplo:
        altitude = 10 + 2 * i  # Por ejemplo, 10, 12, 14, ...
        move_x = 5 if i % 2 == 0 else -5
        move_y = 5 if i < num_rovers / 2 else -5
        rover = System(mavsdk_server_address="localhost", port=port)
        tasks.append(run_rover(rover, port, altitude, move_x, move_y))
    await asyncio.gather(*tasks)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Simulación de rovers con MAVSDK")
    parser.add_argument("num_rovers", type=int, help="Número de rovers a simular")
    args = parser.parse_args()
    asyncio.run(main(args.num_rovers))

