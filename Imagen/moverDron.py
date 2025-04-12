import asyncio
import socket
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed, PositionNedYaw)

SERVER_IP = "127.0.0.1"
SERVER_PORT = 65432

VELOCIDAD_CORRECCION = 0.3  # Velocidad para corregir la alineación (m/s)
VELOCIDAD_AVANCE_LENTO = 0.3 # Velocidad de avance una vez alineado
MARGEN_ERROR_X = 2
MARGEN_ERROR_Y = 2

ESTADO_INICIAL = 0
ESTADO_ALINEANDO_X = 1
ESTADO_ALINEANDO_Y = 2
ESTADO_AVANZANDO = 3


estado_actual = ESTADO_INICIAL
async def mover(drone, x, y):
    """
    Controla el dron de forma SECUENCIAL y SIMPLE:
    1. Alinea en eje X hasta que |x| < MARGEN_ERROR_X
    2. Alinea en eje Y hasta que |y| < MARGEN_ERROR_Y
    3. Avanza lentamente hacia adelante.
    Se usa una variable global simple para controlar los pasos de alineación.

    Offsets:
    +,+ -> Arriba izquierda (Mover arriba, izquierda)
    -,+ -> Arriba derecha   (Mover arriba, derecha)
    -,- -> Abajo derecha    (Mover abajo, derecha)
    +,- -> Abajo izquierda   (Mover abajo, izquierda)

    Interpretación para movimiento del Dron:
    x positivo -> Mover dron a la IZQUIERDA (vel_derecha negativa)
    x negativo -> Mover dron a la DERECHA  (vel_derecha positiva)
    y positivo -> Mover dron ARRIBA        (vel_abajo negativa)
    y negativo -> Mover dron ABAJO         (vel_abajo positiva)
    """
    global estado_actual

    # Paso 1: Alinear en Eje X
    if estado_actual == ESTADO_INICIAL or estado_actual == ESTADO_ALINEANDO_X:
        print(f"--- Estado: Alineando X --- Recibido Offset X: {x:.1f}")
        if abs(x) > MARGEN_ERROR_X:
            estado_actual = ESTADO_ALINEANDO_X
            if x > 0:
                velocidad_lateral = -VELOCIDAD_CORRECCION
                print(f"Moviendo IZQUIERDA ({velocidad_lateral=})")
            else:
                velocidad_lateral = VELOCIDAD_CORRECCION
                print(f"Moviendo DERECHA ({velocidad_lateral=})")

            # VelocityBodyYawspeed(Adelante, Derecha, Abajo, Giro)
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, velocidad_lateral, 0.0, 0.0)
            )
            return

        else: #EJE X ALINEADO
            print(f"Eje X Alineado (|{x:.1f}| <= {MARGEN_ERROR_X}). Pasando a Eje Y.")
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
            )
            await asyncio.sleep(1)
            estado_actual = ESTADO_ALINEANDO_Y


    # Paso 2: Alinear en Eje Y
    if estado_actual == ESTADO_ALINEANDO_Y:
        print(f"--- Estado: Alineando Y --- Recibido Offset Y: {y:.1f}")
        if abs(y) > MARGEN_ERROR_Y:
            if y > 0:
                velocidad_vertical = -VELOCIDAD_CORRECCION 
                print(f"Moviendo ARRIBA ({velocidad_vertical=})")
            else:
                velocidad_vertical = VELOCIDAD_CORRECCION
                print(f"Moviendo ABAJO ({velocidad_vertical=})")

            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, velocidad_vertical, 0.0)
            )
            return 

        else: #EJE X ALINEADO
            print(f"Eje Y Alineado (|{y:.1f}| <= {MARGEN_ERROR_Y}). Pasando a Avanzar.")
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
            )
            await asyncio.sleep(1)
            estado_actual = ESTADO_AVANZANDO

    # Paso 3: Avanzar (Ambos ejes alineados)
    if estado_actual == ESTADO_AVANZANDO:
        print(f"--- Estado: Avanzando --- Velocidad: {VELOCIDAD_AVANCE_LENTO}")
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(VELOCIDAD_AVANCE_LENTO, 0.0, 0.0, 0.0)
        )
        # EL DRON AVANZA HACIA ADELANTE. FUNCIÓN PARA PARAR NO IMPLEMENTADA

# Función para recibir posiciones desde el socket
async def recibir_posiciones(drone, sock):
    loop = asyncio.get_event_loop()
    while True:
        try:
            data, addr = await loop.run_in_executor(None, sock.recvfrom, 1024)
            mensaje = data.decode()
            print(f"Recibido: {mensaje}")

            x_str, y_str = mensaje.split(",")
            x, y = float(x_str), float(y_str)

            await mover(drone, x, y)

        except BlockingIOError:
            await asyncio.sleep(0.1)
            continue

async def run():

    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Esperando conexión con el dron...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Conectado al dron!")
            break

    print("Esperando posición global del dron...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Posición global OK")
            break

    # Crear socket para recibir posiciones
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    sock.bind((SERVER_IP, SERVER_PORT))
    loop = asyncio.get_running_loop()

    print("-- Armando dron")
    await drone.action.arm()

    print("-- Configurando setpoint inicial")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

    print("-- Iniciando modo offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Error al iniciar modo offboard: {error._result.result}")
        print("-- Desarmando dron")
        await drone.action.disarm()
        return

    print("-- Despegandooo")
    await drone.action.takeoff()

    print("-- Iniciando recepción de posiciones")
    await recibir_posiciones(drone, sock)

    print("-- Finalizando movimientos")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(8)

    print("-- Deteniendo modo offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Error al detener modo offboard: {error._result.result}")
        
    print(f"--- Dron aterrizando")
    await drone.action.land()

if __name__ == "__main__":
    # Ejecutar el loop de asyncio
    asyncio.run(run())
