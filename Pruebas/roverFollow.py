"""VERSION 2.0  
UN ROVER REALIZA UN RECORRIDO Y OTRO REALIZA EL MISMO RECORRIDO
 DESDE UNA POSICION INICIAL DIFERENTE LEYENDO LAS COORDENADAS DEL PRIMERO"""

"""Primero arrancar QGroundControl y 2 servidores de mavsdk
/home/alvaro/.local/lib/python3.10/site-packages/mavsdk/bin/mavsdk_server -p 50040 udp://:14541
/home/alvaro/.local/lib/python3.10/site-packages/mavsdk/bin/mavsdk_server -p 50041 udp://:14542

Posteriormente 2 instancias de rovers en PX4;
PX4_SYS_AUTOSTART=4009 PX4_GZ_MODEL_POSE="1,1" PX4_SIM_MODEL=gz_r1_rover ./build/px4_sitl_default/bin/px4 -i 1
PX4_SYS_AUTOSTART=4009 PX4_GZ_MODEL_POSE="2,2" PX4_SIM_MODEL=gz_r1_rover ./build/px4_sitl_default/bin/px4 -i 2

Finalmente ejecutar este archivo; python3 followRover.py 
y posteriormente ejecutar el receiver.py

Se puede comprobar el tráfico de los puertos con 
sudo tcpdump -i any port 14540"""
 
import asyncio
import math
import socket
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)


# Configuración del socket (IP del servidor y puerto)
SERVER_IP = "127.0.0.1"  # Dirección local (localhost)
SERVER_PORT = 65432       # Puerto donde enviará los datos

async def send_position(rover):
    """ Envía la posición del rover cada 2 segundos a un servidor UDP """
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        while True:
            async for position in rover.telemetry.position_velocity_ned():
                current_x = position.position.north_m
                current_y = position.position.east_m
                message = f"{current_x},{current_y}"
                
                # Enviar mensaje al servidor
                sock.sendto(message.encode(), (SERVER_IP, SERVER_PORT))
                print(f"📤 Posición enviada: {message}")

                await asyncio.sleep(1)  # Enviar posición cada 2 segundos
                break  # Salir del `async for` después de la primera lectura


async def wait_until_position_reached(rover, target_x, target_y, angle):
    tolerance = 0.5
    check_interval = 0.01  # Esperar más tiempo entre chequeos

    await rover.offboard.set_position_ned(PositionNedYaw(target_x, target_y, -10.0, angle))

    while True:
        async for position in rover.telemetry.position_velocity_ned():
            current_x = position.position.north_m
            current_y = position.position.east_m

            distance = ((current_x - target_x) ** 2 + (current_y - target_y) ** 2) ** 0.5
            print(f"📍 Rover \"lider\" en ({current_x:.2f}, {current_y:.2f}), distancia a destino: {distance:.2f}m")

            if distance <= tolerance:
                print("✅ Posición alcanzada!")
                return
            
            break  # Salir del `async for` después de la primera lectura

        await asyncio.sleep(check_interval)  # Esperar más tiempo antes de comprobar otra vez


async def run_rover(rover, port):
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
        await rover.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -10.0, 0.0))
        await rover.offboard.start()
        print(f"Rover en puerto {port} en modo Offboard")
    except OffboardError as e:
        print(f"Error en Offboard para rover {port}: {e}")
        return

    # Moverse en metros respecto a la posición inicial
    print(f"Rover en puerto {port} moviéndose a (0,15) metros")
    #await rover.offboard.set_position_ned(PositionNedYaw(0.0, 15.0, -10.0, 90.0))
    #await asyncio.sleep(1)
    await wait_until_position_reached(rover, 0.0, 15.0, 90.0)

    
    for i in range(3):
        print(f"Empezamos la vuelta {i}")

        # Moverse en metros respecto a la posición inicial
        print(f"Rover en puerto {port} moviéndose a (15,15) metros")
        #await rover.offboard.set_position_ned(PositionNedYaw(15.0, 15.0, -10.0, 90.0))
        #await asyncio.sleep(1)
        await wait_until_position_reached(rover, 15.0, 15.0, 90.0)


        # **🔄 Girar en semicírculo desde (15,15)**
        print("🔄 Iniciando giro semicircular...")
        radius = 7.5  # Radio del semicírculo
        center_x, center_y = 15.0, 7.5  # El centro del semicírculo está en (15,7.5)

        # El rover comienza en (15,15) y hace un giro de 180 grados (semicírculo) hacia abajo
        for angle in range(0, 181, 10):  # Gira de 0° a 180° en pasos de 10°
            radians = math.radians(angle)
            # Calcular el nuevo punto en el arco
            move_x = center_x + radius * math.sin(radians)  # Movimiento en X
            move_y = center_y + radius * math.cos(radians)  # Movimiento en Y

            print(f"Moviéndose a ({move_x:.2f}, {move_y:.2f}) con orientación {angle}°")
            #await rover.offboard.set_position_ned(PositionNedYaw(move_x, move_y, -10.0, angle))
            #await asyncio.sleep(1)
            await wait_until_position_reached(rover, move_x, move_y, angle)

        print("✅ Semicírculo completado!")
        
        # Volver a la posición inicial
        print(f"Rover en puerto {port} regresando al punto de inicio")
        #await rover.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -10.0, 0.0))
        #await asyncio.sleep(1)
        await wait_until_position_reached(rover, 0.0, 0.0, 0.0)


        # **🔄 Girar en semicírculo desde (0,0)**
        print("🔄 Iniciando giro semicircular...")
        radius = 7.5  # Radio del semicírculo
        center_x, center_y = 0.0, 7.5  # El centro del semicírculo está en (0,0)

        # El rover comienza en (0,0) y hace un giro de 180 grados (semicírculo) hacia arriba
        for angle in range(180, 361, 10):
            radians = math.radians(angle)
            move_x = center_x + radius * math.sin(radians)
            move_y = center_y + radius * math.cos(radians)

            print(f"Moviéndose a ({move_x:.2f}, {move_y:.2f}) con orientación {angle}°")
            #await rover.offboard.set_position_ned(PositionNedYaw(move_x, move_y, -10.0, angle))
            #await asyncio.sleep(1)
            await wait_until_position_reached(rover, move_x, move_y, angle)

        print("✅ Segundo semicírculo completado!")

        await asyncio.sleep(5)

    await rover.offboard.stop()
    print(f"Rover en puerto {port} saliendo de Offboard")

    await rover.action.land()  # Aunque el rover no aterrice, mantenemos la lógica de aterrizaje por la simulación
    print(f"Rover en puerto {port} aterrizando...")

async def main():
    rover1 = System(mavsdk_server_address="localhost", port=50040)

    # Primero conectamos el rover y esperamos a que esté listo
    await rover1.connect()
    async for state in rover1.core.connection_state():
        if state.is_connected:
            print("✅ Rover conectado correctamente")
            break

    # Luego lanzamos las tareas en paralelo
    await asyncio.gather(
        run_rover(rover1, 50040),
        send_position(rover1)  # Envía la posición en paralelo
    )

if __name__ == "__main__":
    asyncio.run(main())