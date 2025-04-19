import asyncio
import socket
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

SERVER_IP = "127.0.0.1"
SERVER_PORT = 65432
VELOCIDAD_CORRECCION_X = 0.4
VELOCIDAD_CORRECCION_Y = 0.4
VELOCIDAD_AVANCE = 1.0
MARGEN_ERROR_X = 15
MARGEN_ERROR_Y = 15
MARGEN_ERROR_X_ALINEADO = 25
MARGEN_ERROR_Y_ALINEADO = 25
CONTADOR_PERDIDO_MAX = 10 


ESTADO_INICIO = 0
ESTADO_BUSCANDO_PUERTA_1 = 1
ESTADO_ALINEANDO_PUERTA_1 = 2
ESTADO_AVANZANDO_PUERTA_1 = 3
ESTADO_BUSCANDO_PUERTA_2 = 4
ESTADO_ALINEANDO_PUERTA_2 = 5
ESTADO_AVANZANDO_PUERTA_2 = 6
ESTADO_MISION_COMPLETA = 7

estado_nombres = {
    ESTADO_INICIO: "INICIO", ESTADO_BUSCANDO_PUERTA_1: "BUSCANDO_P1",
    ESTADO_ALINEANDO_PUERTA_1: "ALINEANDO_P1", ESTADO_AVANZANDO_PUERTA_1: "AVANZANDO_P1",
    ESTADO_BUSCANDO_PUERTA_2: "BUSCANDO_P2", ESTADO_ALINEANDO_PUERTA_2: "ALINEANDO_P2",
    ESTADO_AVANZANDO_PUERTA_2: "AVANZANDO_P2", ESTADO_MISION_COMPLETA: "COMPLETA"
}

estado_actual = ESTADO_INICIO
objetivo_perdido_contador = 0

async def cambiar_estado(nuevo_estado, drone):
    """Cambia estado, loggea y detiene dron si es necesario."""
    global estado_actual, objetivo_perdido_contador
    if estado_actual != nuevo_estado:
        print(f"Estado: {estado_nombres[estado_actual]} -> {estado_nombres[nuevo_estado]}")
        estado_actual = nuevo_estado
        objetivo_perdido_contador = 0
        if nuevo_estado in [ESTADO_BUSCANDO_PUERTA_2, ESTADO_MISION_COMPLETA]:
             await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
             await asyncio.sleep(0.2) 

async def mover(drone, offset_x, offset_y, distancia, num_targets):
    """Controla movimiento y estados basado en datos recibidos."""
    global estado_actual, objetivo_perdido_contador

    velocidad_avance, velocidad_lateral, velocidad_vertical = 0.0, 0.0, 0.0
    target_detected = num_targets > 0 and distancia > 0

    if estado_actual == ESTADO_BUSCANDO_PUERTA_1:
        if target_detected: await cambiar_estado(ESTADO_ALINEANDO_PUERTA_1, drone)

    elif estado_actual == ESTADO_ALINEANDO_PUERTA_1:
        if not target_detected:
            objetivo_perdido_contador += 1
            if objetivo_perdido_contador >= CONTADOR_PERDIDO_MAX:
                print("WARN: P1 perdida alineando, reintentando búsqueda.")
                await cambiar_estado(ESTADO_BUSCANDO_PUERTA_1, drone)
        else:
            objetivo_perdido_contador = 0
            alineado_x = abs(offset_x) <= MARGEN_ERROR_X
            alineado_y = abs(offset_y) <= MARGEN_ERROR_Y
            if not alineado_x: velocidad_lateral = -VELOCIDAD_CORRECCION_X if offset_x < 0 else VELOCIDAD_CORRECCION_X
            if not alineado_y: velocidad_vertical = VELOCIDAD_CORRECCION_Y if offset_y > 0 else -VELOCIDAD_CORRECCION_Y
            if alineado_x and alineado_y:
                await cambiar_estado(ESTADO_AVANZANDO_PUERTA_1, drone)

    elif estado_actual == ESTADO_AVANZANDO_PUERTA_1:
        if not target_detected:
            objetivo_perdido_contador += 1
            if objetivo_perdido_contador >= CONTADOR_PERDIDO_MAX:
                print("INFO: P1 pasada.")
                await cambiar_estado(ESTADO_BUSCANDO_PUERTA_2, drone)
        else:
            objetivo_perdido_contador = 0
            velocidad_avance = VELOCIDAD_AVANCE
            if abs(offset_x) > MARGEN_ERROR_X_ALINEADO: velocidad_lateral = -VELOCIDAD_CORRECCION_X if offset_x < 0 else VELOCIDAD_CORRECCION_X
            if abs(offset_y) > MARGEN_ERROR_Y_ALINEADO: velocidad_vertical = VELOCIDAD_CORRECCION_Y if offset_y > 0 else -VELOCIDAD_CORRECCION_Y

    elif estado_actual == ESTADO_BUSCANDO_PUERTA_2:
        if target_detected: await cambiar_estado(ESTADO_ALINEANDO_PUERTA_2, drone)

    elif estado_actual == ESTADO_ALINEANDO_PUERTA_2:
        if not target_detected:
            objetivo_perdido_contador += 1
            if objetivo_perdido_contador >= CONTADOR_PERDIDO_MAX:
                print("WARN: P2 perdida alineando. Misión finalizada.")
                await cambiar_estado(ESTADO_MISION_COMPLETA, drone)
        else:
            objetivo_perdido_contador = 0
            alineado_x = abs(offset_x) <= MARGEN_ERROR_X
            alineado_y = abs(offset_y) <= MARGEN_ERROR_Y
            if not alineado_x: velocidad_lateral = -VELOCIDAD_CORRECCION_X if offset_x < 0 else VELOCIDAD_CORRECCION_X
            if not alineado_y: velocidad_vertical = VELOCIDAD_CORRECCION_Y if offset_y > 0 else -VELOCIDAD_CORRECCION_Y
            if alineado_x and alineado_y:
                await cambiar_estado(ESTADO_AVANZANDO_PUERTA_2, drone)

    elif estado_actual == ESTADO_AVANZANDO_PUERTA_2:
        if not target_detected:
            objetivo_perdido_contador += 1
            if objetivo_perdido_contador >= CONTADOR_PERDIDO_MAX:
                print("INFO: P2 pasada. Misión finalizada.")
                await cambiar_estado(ESTADO_MISION_COMPLETA, drone)
        else:
            objetivo_perdido_contador = 0
            velocidad_avance = VELOCIDAD_AVANCE
            if abs(offset_x) > MARGEN_ERROR_X_ALINEADO: velocidad_lateral = -VELOCIDAD_CORRECCION_X if offset_x < 0 else VELOCIDAD_CORRECCION_X
            if abs(offset_y) > MARGEN_ERROR_Y_ALINEADO: velocidad_vertical = VELOCIDAD_CORRECCION_Y if offset_y > 0 else -VELOCIDAD_CORRECCION_Y

    elif estado_actual == ESTADO_MISION_COMPLETA:
        print("Mision completa")

    if estado_actual != ESTADO_MISION_COMPLETA:
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(velocidad_avance, velocidad_lateral, velocidad_vertical, 0.0)
        )

async def recibir_posiciones(drone, sock):
    """Bucle principal: recibe UDP y llama a mover()."""
    loop = asyncio.get_event_loop()
    global estado_actual
    print(f"--- Estado Inicial: {estado_nombres[estado_actual]} ---")

    while estado_actual != ESTADO_MISION_COMPLETA:
        try:
            data, addr = await loop.run_in_executor(None, sock.recvfrom, 1024)
            mensaje = data.decode('utf-8')
            try:
                x_str, y_str, dist_str, num_t_str = mensaje.split(',')
                offset_x, offset_y = float(x_str), float(y_str)
                distancia, num_targets = float(dist_str), int(num_t_str)
                await mover(drone, offset_x, offset_y, distancia, num_targets)
            except ValueError:
                print(f"WARN: Mensaje UDP inválido: '{mensaje}'")
                continue
        except Exception as e:
             print(f"ERROR en recibir_posiciones: {e}. Finalizando misión.")
             await cambiar_estado(ESTADO_MISION_COMPLETA, drone)
             break 

    print("--- Bucle principal finalizado ---")

async def run():
    """Función principal: conecta, prepara y ejecuta la misión."""
    drone = System()
    sock = None
    global estado_actual
    estado_actual = ESTADO_BUSCANDO_PUERTA_1 # Estado inicial

    try:
        print("Conectando...")
        await drone.connect(system_address="udp://:14540")
        async for state in drone.core.connection_state():
            if state.is_connected: print("-- Conectado."); break
        async for health in drone.telemetry.health():
            if health.is_global_position_ok: print("-- GPS OK."); break # Solo GPS para simplificar
            await asyncio.sleep(1)

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((SERVER_IP, SERVER_PORT))
        print(f"-- Socket UDP en {SERVER_IP}:{SERVER_PORT}")

        print("-- Armando")
        await drone.action.arm()
        print("-- Estableciendo setpoint inicial")
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        print("-- Iniciando Offboard")
        try:
            await drone.offboard.start()
        except OffboardError as error:
            print(f"FATAL: Error al iniciar Offboard: {error}. Desarmando.")
            await drone.action.disarm() 
            if sock: sock.close()
            return 

        print("-- Despegue")
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, -1.5, 0.0))
        await asyncio.sleep(4)
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        print("-- Altura alcanzada. Iniciando misión.")

        # Bucle Principal
        await recibir_posiciones(drone, sock)

    except Exception as e:
        print(f"ERROR INESPERADO en run(): {e}")
        import traceback
        traceback.print_exc()
    finally:
        # --- Finalización Segura ---
        print("--- Finalizando Misión (Bloque Finally) ---")
        print("-- Deteniendo velocidad")
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.5)

        print("-- Deteniendo Offboard")
        try: 
            await drone.offboard.stop()
        except OffboardError as error:
            print(f"WARN: Fallo al detener Offboard: {error}")

        print("-- Aterrizando")
        try:
            await drone.action.land()
            await asyncio.sleep(8)
        except Exception as e:
            print(f"WARN: Fallo al aterrizar ({e}). Intentando desarmar.")
            try: 
                await drone.action.disarm()
            except Exception as disarm_e:
                print(f"WARN: Fallo al desarmar: {disarm_e}")

        if sock: sock.close(); print("-- Socket cerrado.")
        print("--- Secuencia de finalización completada. ---")

if __name__ == "__main__":
    print("Iniciando script de control (simplificado)...")
    asyncio.run(run())
    print("Script de control (simplificado) finalizado.")