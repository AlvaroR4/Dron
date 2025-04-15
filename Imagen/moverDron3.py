import asyncio
import socket
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed, PositionNedYaw)

SERVER_IP = "127.0.0.1"
SERVER_PORT = 65432

VELOCIDAD_CORRECCION_X = 0.5
VELOCIDAD_CORRECCION_Y = 0.5
VELOCIDAD_AVANCE = 1.5
MARGEN_ERROR_X = 3
MARGEN_ERROR_Y = 3
MARGEN_ERROR_X_ALINEADO = 10
MARGEN_ERROR_Y_ALINEADO = 10

ESTADO_INICIAL = 0
ESTADO_ALINEANDO_X = 1
ESTADO_ALINEANDO_Y = 2
ESTADO_AVANZANDO = 3

estado_actual = ESTADO_INICIAL

async def mover(drone, x, y):
    global estado_actual


    if estado_actual == ESTADO_INICIAL or estado_actual == ESTADO_ALINEANDO_X:
        print(f"--- Estado: Alineando X --- Recibido Offset X: {x:.1f}")
        if abs(x) > MARGEN_ERROR_X:
            estado_actual = ESTADO_ALINEANDO_X
            if x < 0:
                velocidad_lateral = -VELOCIDAD_CORRECCION_X
                print(f"Moviendo IZQUIERDA ({velocidad_lateral=})")
            else:
                velocidad_lateral = VELOCIDAD_CORRECCION_X
                print(f"Moviendo DERECHA ({velocidad_lateral=})")
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, velocidad_lateral, 0.0, 0.0))
            return
        else:
            print(f"Eje X Alineado (|{x:.1f}| <= {MARGEN_ERROR_X}). Pasando a Eje Y.")
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(0.5)
            estado_actual = ESTADO_ALINEANDO_Y

    if estado_actual == ESTADO_ALINEANDO_Y:
        print(f"--- Estado: Alineando Y --- Recibido Offset Y: {y:.1f}")
        if abs(y) > MARGEN_ERROR_Y:
            estado_actual = ESTADO_ALINEANDO_Y
            if y < 0:
                velocidad_vertical = -VELOCIDAD_CORRECCION_Y
                print(f"Moviendo ARRIBA ({velocidad_vertical=})")
            else:
                velocidad_vertical = VELOCIDAD_CORRECCION_Y
                print(f"Moviendo ABAJO ({velocidad_vertical=})")
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, velocidad_vertical, 0.0))
            return
        else:
            print(f"Eje Y Alineado (|{y:.1f}| <= {MARGEN_ERROR_Y}). Pasando a Avanzar.")
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(0.5)
            estado_actual = ESTADO_AVANZANDO

    if estado_actual == ESTADO_AVANZANDO:
        velocidad_lateral = 0.0
        velocidad_vertical = 0.0
        print(f"--- Estado: Avanzando --- Velocidad: {VELOCIDAD_AVANCE}")

        if abs(x) > MARGEN_ERROR_X_ALINEADO:
            if x < 0:
                velocidad_lateral = -VELOCIDAD_CORRECCION_X
                print(f"Moviendo IZQUIERDA ({velocidad_lateral=})")
            else:
                velocidad_lateral = VELOCIDAD_CORRECCION_X
                print(f"Moviendo DERECHA ({velocidad_lateral=})")
        
        if abs(y) > MARGEN_ERROR_Y_ALINEADO:
            if y < 0:
                velocidad_vertical = -VELOCIDAD_CORRECCION_Y
                print(f"Moviendo ARRIBA ({velocidad_vertical=})")
            else:
                velocidad_vertical = VELOCIDAD_CORRECCION_Y
                print(f"Moviendo ABAJO ({velocidad_vertical=})")
        
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(VELOCIDAD_AVANCE, velocidad_lateral, velocidad_vertical, 0.0))



async def recibir_posiciones(drone, sock):
    loop = asyncio.get_event_loop()
    global estado_actual
    while True:
        try:
            data, addr = await loop.run_in_executor(None, sock.recvfrom, 1024)
            mensaje = data.decode('utf-8')
            print(f"Recibido: {mensaje}")

            x_str, y_str, area_str = mensaje.split(',')
            x = float(x_str)
            y = float(y_str)
            area = float(area_str)

            await mover(drone, x, y)

            if estado_actual == ESTADO_AVANZANDO and area == 0.0:
                print(f"Alineado y Área ({area:.0f}) == 0. Objetivo pasado. Saliendo del bucle.")
                break

        except BlockingIOError:
            await asyncio.sleep(0.05)
            continue
        except ValueError:
            print(f"Error: Mensaje UDP mal formateado recibido: '{mensaje}'")
            continue
        except Exception as e:
             print(f"Error en recibir_posiciones: {e}")
             break

async def run():
    drone = System()
    sock = None
    global estado_actual
    estado_actual = ESTADO_INICIAL

    try:
        
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

        print("-- Subo")
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, -2.0, 0.0))
        await asyncio.sleep(6)

        print("-- Iniciando modo offboard")
        try:
            await drone.offboard.start()
        except OffboardError as error:
            print(f"Error al iniciar modo offboard: {error._result.result}")
            print("-- Desarmando dron")
            await drone.action.disarm()
            if sock: sock.close()
            return


        print("-- Iniciando recepción...")
        await recibir_posiciones(drone, sock)

        print("Objetivo pasado")
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        )
        await asyncio.sleep(1)


    except Exception as e:
        print(f"Error inesperado: {e}")
    finally:
        print("-- Ejecutando bloque Finally: Deteniendo y Aterrizando...")

        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        )

        print("-- Stopping offboard")
        try:
            await drone.offboard.stop()  # Intenta salir del modo offboard
        except OffboardError as error:  # Muestra error si no puede detenerlo
            print(f"Stopping offboard mode failed with error code: {error._result.result}")

        await drone.action.land()



        if sock:
            sock.close()
            print("-- Socket UDP cerrado.")
        print("-- Secuencia de finalización completada.")

if __name__ == "__main__":
    print("Iniciando ...")
    asyncio.run(run())
    print("Script finalizado.")