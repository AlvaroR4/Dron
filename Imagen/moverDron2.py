import asyncio
import socket
import numpy as np # Necesario para np.clip
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed, PositionNedYaw)

SERVER_IP = "127.0.0.1"
SERVER_PORT = 65432

KP_X = 0.05  # Ganancia proporcional para el eje X
KP_Y = 0.05  # Ganancia proporcional para el eje Y 

MAX_VEL_LATERAL = 5
MAX_VEL_VERTICAL = 5

UMBRAL_AVANCE_X = 5
UMBRAL_AVANCE_Y = 5   
VELOCIDAD_AVANCE = 2  

async def mover_proporcional(drone: System, x: float, y: float):
    """
    Controla el dron usando control proporcional basado en offsets (x, y).
    Intenta alinear X e Y simultáneamente. Avanza si el error es pequeño.

    Interpretación de Offsets (asumida por el código original):
     x: Offset horizontal desde el centro. x > 0 -> objetivo a la DERECHA -> mover DERECHA (vel_lateral +)
                                          x < 0 -> objetivo a la IZQUIERDA -> mover IZQUIERDA (vel_lateral -)
     y: Offset vertical desde el centro.  y > 0 -> objetivo ABAJO -> mover ABAJO (vel_vertical +)
                                          y < 0 -> objetivo ARRIBA -> mover ARRIBA (vel_vertical -)

    """
    print(f"--- Control Proporcional --- Offsets Recibidos: X={x:.1f}, Y={y:.1f}")

    vel_lateral = KP_X * x
    vel_vertical = KP_Y * y

    vel_lateral = np.clip(vel_lateral, -MAX_VEL_LATERAL, MAX_VEL_LATERAL)
    vel_vertical = np.clip(vel_vertical, -MAX_VEL_VERTICAL, MAX_VEL_VERTICAL)

    if abs(x) < UMBRAL_AVANCE_X and abs(y) < UMBRAL_AVANCE_Y:
        vel_forward = VELOCIDAD_AVANCE
        print(f"    Alineado (|x|<{UMBRAL_AVANCE_X}, |y|<{UMBRAL_AVANCE_Y}). Avanzando.")
    else:
        vel_forward = 0.0
        print(f"    Alineando (|x| o |y| fuera de umbral). No avanza.")

    print(f"    Enviando Velocidad Cuerpo: Adelante={vel_forward:.2f}, Derecha={vel_lateral:.2f}, Abajo={vel_vertical:.2f}")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(vel_forward, vel_lateral, vel_vertical, 0.0))



async def recibir_posiciones(drone, sock):
    loop = asyncio.get_event_loop()
    print("-- Iniciando bucle de recepción de posiciones UDP...")
    while True:
        data, addr = await loop.run_in_executor(None, sock.recvfrom, 1024)
        mensaje = data.decode('utf-8')

        x_str, y_str = mensaje.split(',')
        x = float(x_str)
        y = float(y_str)

        await mover_proporcional(drone, x, y)


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Esperando conexión con el dron...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Conectado al dron!")
            break
    else: 
        print("Error: No se pudo conectar al dron.")
        return

    print("Esperando posición global del dron...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Posición global OK")
            break
    else:
        print("Error: GPS del dron no listo.")
        return

    sock = None 
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(False) 
        sock.bind((SERVER_IP, SERVER_PORT))
        print(f"-- Socket UDP escuchando en {SERVER_IP}:{SERVER_PORT}")
    except Exception as sock_err:
        print(f"Error al crear o bindear el socket UDP: {sock_err}")
        return 

    print("-- Armando dron")
    await drone.action.arm()
    await asyncio.sleep(1)

    print("-- Configurando setpoint inicial (velocidad 0)")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(0.1)

    print("-- Iniciando modo offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Error al iniciar modo offboard: {error._result.result}")
        print("-- Desarmando dron")
        await drone.action.disarm()
        if sock: sock.close() 
        return

    print("-- Despegando...")
    await drone.action.takeoff()
    await asyncio.sleep(10)



    try:
        await recibir_posiciones(drone, sock)
    except asyncio.CancelledError:
         print("Bucle de recepción cancelado.")
    finally:
        print("-- Finalizando movimientos y deteniendo offboard...")
        try:
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(1)
            await drone.offboard.stop()
            print("-- Modo offboard detenido.")
        except OffboardError as error:
            print(f"Error al detener modo offboard: {error._result.result}")

        print(f"-- Dron aterrizando...")
        await drone.action.land()
        await asyncio.sleep(5)

        print("-- Desarmando.")
        await drone.action.disarm()

        if sock:
            sock.close()
            print("-- Socket UDP cerrado.")

if __name__ == "__main__":
    print("Iniciando script de control de dron proporcional...")
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        print("Script interrumpido por el usuario (Ctrl+C).")
    print("Script finalizado.")