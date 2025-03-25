"""Primero arrancar QGroundControl y 3 instancias de rovers en PX4;
PX4_SYS_AUTOSTART=4009 PX4_GZ_MODEL_POSE="1,1" PX4_SIM_MODEL=gz_r1_rover ./build/px4_sitl_default/bin/px4 -i 1
PX4_SYS_AUTOSTART=4009 PX4_GZ_MODEL_POSE="2,1" PX4_SIM_MODEL=gz_r1_rover ./build/px4_sitl_default/bin/px4 -i 2
PX4_SYS_AUTOSTART=4009 PX4_GZ_MODEL_POSE="3,1" PX4_SIM_MODEL=gz_r1_rover ./build/px4_sitl_default/bin/px4 -i 3

Se puede comprobar el tráfico de los puertos con 
sudo tcpdump -i any port 14540

Comprobar con mavlink status los puertos, pero teoricamente se conectaras en 14541,14542,14543
Arrancar 3 mavsdk_server;
/home/alvaro/.local/lib/python3.10/site-packages/mavsdk/bin/mavsdk_server -p 50040 udp://:14541
/home/alvaro/.local/lib/python3.10/site-packages/mavsdk/bin/mavsdk_server -p 50041 udp://:14542
/home/alvaro/.local/lib/python3.10/site-packages/mavsdk/bin/mavsdk_server -p 50042 udp://:14543

Finalmente ejecutar este archivo; python3 3rovers.py """
 
import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)

async def run_rover(rover, port, altitude, move_x, move_y):
    """ Conecta, arma, despega, mueve en metros y aterriza un rover """
    await rover.connect()

    print(f"Rover en puerto {port} conectando...")
    async for state in rover.core.connection_state():
        if state.is_connected:
            print(f"Rover en puerto {port} conectado")
            break

    await rover.action.arm()
    print(f"Rover en puerto {port} armado")

    await rover.action.takeoff()  # Aunque sea un rover, para efectos de simulación seguimos con despegue
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

    await rover.action.land()  # Aunque el rover no aterrice, mantenemos la lógica de aterrizaje por la simulación
    print(f"Rover en puerto {port} aterrizando...")

async def main():
    rover1 = System(mavsdk_server_address="localhost", port=50040)
    rover2 = System(mavsdk_server_address="localhost", port=50041)
    rover3 = System(mavsdk_server_address="localhost", port=50042)

    await asyncio.gather(
        run_rover(rover1, 50040, 10, 5, 5),  # Se mueve 5m en X y 5m en Y
        run_rover(rover2, 50041, 12, 5, 5), # Se mueve 5m en X y 5m en Y
        run_rover(rover3, 50042, 20, 5, -5)  # Se mueve 5m en X y -5m en Y
    )

if __name__ == "__main__":
    asyncio.run(main())

