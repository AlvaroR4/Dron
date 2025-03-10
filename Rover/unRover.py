#/home/alvaro/.local/lib/python3.10/site-packages/mavsdk/bin/mavsdk_server -p 50050 udp://:14540

#PX4_SYS_AUTOSTART=4009 PX4_GZ_MODEL_POSE="1,1" PX4_SIM_PORT=14540 PX4_SIM_MODEL=gz_r1_rover ./build/px4_sitl_default/bin/px4 -i 1



import asyncio
import mavsdk

from mavsdk import System

async def run(rover):
    print("Esperando conexión...")
    await rover.connect(system_address="udp://:14540")
	
    print("Esperando conexión...")
    async for state in rover.core.connection_state():
        if state.is_connected:
            print(f"Conectado al rover.")
            break

    # Ar#mar el rover
    print("Armando el rover...")
    await rover.action.arm()

    # Mover hacia adelante
    print("Moviendo hacia adelante...")
    await rover.manual_control.set_manual_control_input(0.5, 0.0, 0.0, 0.5)
    await asyncio.sleep(20)
    
    #await rover.manual_control.set_manual_control_input(0.5, 0.0, 0.0, 0.5)
    #await asyncio.sleep(20)

    # Parar el rover
    print("Deteniendo el rover...")
    await rover.manual_control.set_manual_control_input(0.0, 0.0, 0.0, 0.0)

    print("Desarmando el rover...")
    await rover.action.disarm()
    


grpc_port = 50050
rover = System(mavsdk_server_address="localhost", port=50050)

asyncio.ensure_future(run(rover))
asyncio.get_event_loop().run_forever()

#if __name__ == "__main__":
#    asyncio.run(main(rover))
