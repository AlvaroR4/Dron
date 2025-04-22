import asyncio
import socket
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed, PositionNedYaw)

SERVER_IP = "127.0.0.1"         # Dirección IP donde este script escucha
SERVER_PORT = 65432           # Puerto UDP donde este script escucha los datos de la imagen

VELOCIDAD_CORRECCION_X = 0.4  # Velocidad (m/s) para corregir errores laterales (eje Y del dron, izq/der)
VELOCIDAD_CORRECCION_Y = 0.4  # Velocidad (m/s) para corregir errores verticales (eje Z del dron, arriba/abajo)
VELOCIDAD_AVANCE = 1.0        # Velocidad (m/s) constante hacia adelante al atravesar (eje X del dron)

MARGEN_ERROR_X = 5           # Error máx. en píxeles (offset X) para considerar alineado antes de avanzar
MARGEN_ERROR_Y = 5           # Error máx. en píxeles (offset Y) para considerar alineado antes de avanzar
MARGEN_ERROR_X_ALINEADO = 5  # Margen de error X más amplio permitido mientras se avanza
MARGEN_ERROR_Y_ALINEADO = 5  # Margen de error Y más amplio permitido mientras se avanza

CONTADOR_PERDIDO_MAX = 10     # Num de ciclos seguidos sin detectar objetivo para confirmar pérdida/paso
DISTANCIA_UMBRAL_CERCA = 1.7  # Distancia (m) umbral para considerar que estamos "cerca" de la puerta (Dará comienzo a fase_avance) 
UMBRAL_AUMENTO_DIST = 0.75    # Aumento de distancia (m) necesario tras estar "cerca" para confirmar paso


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
min_distancia_vista = float('inf')
fase_avance = 'INICIAL'


async def cambiar_estado(nuevo_estado, drone):
    """Cambia estado, loggea, resetea variables y detiene dron si es necesario."""
    global estado_actual, objetivo_perdido_contador, min_distancia_vista, fase_avance
    if estado_actual != nuevo_estado:
        print(f"Estado: {estado_nombres[estado_actual]} -> {estado_nombres[nuevo_estado]}")
        estado_actual = nuevo_estado
        objetivo_perdido_contador = 0

        if nuevo_estado in [ESTADO_ALINEANDO_PUERTA_1, ESTADO_ALINEANDO_PUERTA_2]:
            min_distancia_vista = float('inf')
            fase_avance = 'INICIAL'

        if nuevo_estado in [ESTADO_BUSCANDO_PUERTA_2, ESTADO_MISION_COMPLETA]:
             await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
             await asyncio.sleep(0.2)


async def mover(drone, offset_x, offset_y, distancia, num_targets):
    """Controla movimiento y estados basado en datos recibidos y tendencia de distancia."""
    global estado_actual, objetivo_perdido_contador, min_distancia_vista, fase_avance

    velocidad_avance, velocidad_lateral, velocidad_vertical = 0.0, 0.0, 0.0
    target_detected = num_targets > 0 and distancia > 0 # Condición para buscar inicialmente

    if estado_actual == ESTADO_BUSCANDO_PUERTA_1:
        if target_detected:
            await cambiar_estado(ESTADO_ALINEANDO_PUERTA_1, drone)
            # El reseteo de min_distancia y fase_avance ocurre en cambiar_estado

    elif estado_actual == ESTADO_ALINEANDO_PUERTA_1:
        if not target_detected: # Si perdemos el target mientras alineamos
            objetivo_perdido_contador += 1
            if objetivo_perdido_contador >= CONTADOR_PERDIDO_MAX:
                print("WARN: P1 perdida alineando, reintentando búsqueda.")
                await cambiar_estado(ESTADO_BUSCANDO_PUERTA_1, drone)
        else: # Si vemos el target, alineamos
            objetivo_perdido_contador = 0
            alineado_x = abs(offset_x) <= MARGEN_ERROR_X
            alineado_y = abs(offset_y) <= MARGEN_ERROR_Y
            if not alineado_x: velocidad_lateral = -VELOCIDAD_CORRECCION_X if offset_x < 0 else VELOCIDAD_CORRECCION_X
            if not alineado_y: velocidad_vertical = VELOCIDAD_CORRECCION_Y if offset_y > 0 else -VELOCIDAD_CORRECCION_Y
            if alineado_x and alineado_y:
                await cambiar_estado(ESTADO_AVANZANDO_PUERTA_1, drone)

    elif estado_actual == ESTADO_AVANZANDO_PUERTA_1:
        puerta_pasada = False # Flag para saber si hemos detectado paso
        if target_detected:
            objetivo_perdido_contador = 0

            min_distancia_vista = min(min_distancia_vista, distancia)
            if distancia < DISTANCIA_UMBRAL_CERCA:
                if fase_avance == 'INICIAL': print(f"--- Fase Avance P1: CERCA (Dist: {distancia:.2f}m < {DISTANCIA_UMBRAL_CERCA}m) ---")
                fase_avance = 'CERCA'

            # Condición de paso 1: Distancia aumenta tras estar cerca
            if fase_avance == 'CERCA' and distancia > min_distancia_vista + UMBRAL_AUMENTO_DIST:
                print(f"INFO: P1 pasada (Detectado aumento distancia: {distancia:.2f}m > min {min_distancia_vista:.2f}m + {UMBRAL_AUMENTO_DIST}m)")
                puerta_pasada = True
            

            if not puerta_pasada:
                # Si no hemos detectado paso por distancia, seguimos avanzando y corrigiendo
                velocidad_avance = VELOCIDAD_AVANCE
                if abs(offset_x) > MARGEN_ERROR_X_ALINEADO: velocidad_lateral = -VELOCIDAD_CORRECCION_X if offset_x < 0 else VELOCIDAD_CORRECCION_X
                if abs(offset_y) > MARGEN_ERROR_Y_ALINEADO: velocidad_vertical = VELOCIDAD_CORRECCION_Y if offset_y > 0 else -VELOCIDAD_CORRECCION_Y

        else: # Target NO detectado
            objetivo_perdido_contador += 1
            print(f"INFO: P1 no detectado ({objetivo_perdido_contador}/{CONTADOR_PERDIDO_MAX}). Comprobando paso...")
            # Aplicamos lógica de detener avance mientras se confirma pérdida
            velocidad_avance = 0.0
            velocidad_lateral = 0.0
            velocidad_vertical = 0.0
            if objetivo_perdido_contador >= CONTADOR_PERDIDO_MAX:
                # Condición de paso 2: Pérdida confirmada. Verificamos si habíamos estado cerca.
                if fase_avance == 'CERCA':
                    print("INFO: P1 pasada (Confirmado por pérdida de objetivo tras estar cerca).")
                    puerta_pasada = True
                else:
                    # Perdimos el objetivo antes de acercarnos lo suficiente
                    print("WARN: P1 perdida antes de confirmar cercanía. Asumiendo paso igualmente.")
                    puerta_pasada = True # Simplificación: asumir paso

        if puerta_pasada:
            await cambiar_estado(ESTADO_BUSCANDO_PUERTA_2, drone)
            # La velocidad ya está a 0 si se pasó por pérdida, o cambiar_estado la pondrá a 0


    elif estado_actual == ESTADO_BUSCANDO_PUERTA_2:
        if target_detected:
            await cambiar_estado(ESTADO_ALINEANDO_PUERTA_2, drone)

    elif estado_actual == ESTADO_ALINEANDO_PUERTA_2:
        if not target_detected:
            objetivo_perdido_contador += 1
            if objetivo_perdido_contador >= CONTADOR_PERDIDO_MAX:
                print("WARN: P2 perdida alineando. Finalizando misión.")
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
        puerta_pasada = False
        if target_detected:
            objetivo_perdido_contador = 0

            min_distancia_vista = min(min_distancia_vista, distancia)
            if distancia < DISTANCIA_UMBRAL_CERCA:
                 if fase_avance == 'INICIAL': print(f"--- Fase Avance P2: CERCA (Dist: {distancia:.2f}m < {DISTANCIA_UMBRAL_CERCA}m) ---")
                 fase_avance = 'CERCA'

            # Condición de paso 1: Distancia aumenta tras estar cerca
            if fase_avance == 'CERCA' and distancia > min_distancia_vista + UMBRAL_AUMENTO_DIST:
                print(f"INFO: P2 pasada (Detectado aumento distancia: {distancia:.2f}m > min {min_distancia_vista:.2f}m + {UMBRAL_AUMENTO_DIST}m)")
                puerta_pasada = True

            if not puerta_pasada:
                velocidad_avance = VELOCIDAD_AVANCE
                if abs(offset_x) > MARGEN_ERROR_X_ALINEADO: velocidad_lateral = -VELOCIDAD_CORRECCION_X if offset_x < 0 else VELOCIDAD_CORRECCION_X
                if abs(offset_y) > MARGEN_ERROR_Y_ALINEADO: velocidad_vertical = VELOCIDAD_CORRECCION_Y if offset_y > 0 else -VELOCIDAD_CORRECCION_Y

        else: # Target NO detectado
            objetivo_perdido_contador += 1
            print(f"INFO: P2 no detectado ({objetivo_perdido_contador}/{CONTADOR_PERDIDO_MAX}). Comprobando paso...")
            velocidad_avance = 0.0
            velocidad_lateral = 0.0
            velocidad_vertical = 0.0
            if objetivo_perdido_contador >= CONTADOR_PERDIDO_MAX:
                # Condición de paso 2: Pérdida confirmada.
                 if fase_avance == 'CERCA':
                     print("INFO: P2 pasada (Confirmado por pérdida de objetivo tras estar cerca). Misión finalizada.")
                     puerta_pasada = True
                 else:
                     print("WARN: P2 perdida antes de confirmar cercanía. Asumiendo misión finalizada.")
                     puerta_pasada = True 

        if puerta_pasada:
            await cambiar_estado(ESTADO_MISION_COMPLETA, drone)

    elif estado_actual == ESTADO_MISION_COMPLETA:
        pass # El bucle principal terminará

    # Solo aplicamos si no hemos llegado al estado final AÚN en esta iteración
    # y si no estamos en un estado donde cambiar_estado ya puso velocidad 0 (BUSCANDO_P2)
    if estado_actual not in [ESTADO_MISION_COMPLETA, ESTADO_BUSCANDO_PUERTA_2]:
         if not (estado_actual == ESTADO_AVANZANDO_PUERTA_1 and not target_detected) and \
            not (estado_actual == ESTADO_AVANZANDO_PUERTA_2 and not target_detected):
             await drone.offboard.set_velocity_body(
                 VelocityBodyYawspeed(velocidad_avance, velocidad_lateral, velocidad_vertical, 0.0)
             )
         else:
             pass


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
    drone = System()
    sock = None
    global estado_actual, min_distancia_vista, fase_avance
    estado_actual = ESTADO_BUSCANDO_PUERTA_1
    min_distancia_vista = float('inf')
    fase_avance = 'INICIAL'

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