""""
moverDron5.py es una mejora de moverDron4.py
Funciona exactamente igual, la diferencia es que busca N puertas en vez de 2 puertas
Por lo tanto solo se llegara a ESTADO_MISION_COMPLETA si se ha pasado un tiempo sin detectar puertas

"""
import asyncio
import socket
import csv
import datetime
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed, PositionNedYaw)

SERVER_IP = "127.0.0.1"         # Dirección IP donde este script escucha
SERVER_PORT = 65432           # Puerto UDP donde este script escucha los datos de la imagen

ARCHIVO_TRAYECTORIA = "trayectoria.csv"
ARCHIVO_PASO_PUERTAS = "paso_puertas.csv"

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
TIEMPO_AVANCE_EXTRA = 2   # Segundos de avance recto tras detectar paso de puerta

ESTADO_INICIO = 0
ESTADO_BUSCANDO = 1
ESTADO_ALINEANDO = 2
ESTADO_AVANZANDO = 3
ESTADO_BUSCANDO_PUERTA_2 = 4
ESTADO_ALINEANDO_PUERTA_2 = 5
ESTADO_AVANZANDO_PUERTA_2 = 6
ESTADO_MISION_COMPLETA = 7
CONTADOR_BUSQUEDA_MAX = 50 # Num máx de ciclos buscando sin encontrar antes de terminar

estado_nombres = {
    ESTADO_INICIO: "INICIO", ESTADO_BUSCANDO: "BUSCANDO_P1",
    ESTADO_ALINEANDO: "ALINEANDO_P1", ESTADO_AVANZANDO: "AVANZANDO_P1",
    ESTADO_BUSCANDO_PUERTA_2: "BUSCANDO_P2", ESTADO_ALINEANDO_PUERTA_2: "ALINEANDO_P2",
    ESTADO_AVANZANDO_PUERTA_2: "AVANZANDO_P2", ESTADO_MISION_COMPLETA: "COMPLETA"
}


estado_actual = ESTADO_INICIO             # Variable que almacena el estado actual de la máquina de estados de la misión.
objetivo_perdido_contador = 0             # Contador de ciclos consecutivos en los que no se detecta el objetivo actual.
min_distancia_vista = float('inf')        # Guarda la distancia mínima detectada a la puerta actual (se resetea para cada puerta).
fase_avance = 'INICIAL'                   # Indica la fase del avance hacia la puerta actual ('INICIAL' o 'CERCA').
puertas_pasadas = 0         # Contador de puertas superadas
ciclos_sin_objetivo_buscando = 0 # Contador para el timeout en búsqueda

#Para guardar posiciones en csv
ultima_posicion_ned = None
log_task = None

def escribir_csv(filename, data_row):
        open(filename, mode='a', newline='', encoding='utf-8')
        writer = csv.writer(filename)
        writer.writerow(data_row)

async def log_trayectoria(drone):
    """Tarea asíncrona que guardda la posición NED periódicamente."""
    global ultima_posicion_ned 
    async for position_ned in drone.telemetry.position_velocity_ned():
        ultima_posicion_ned = position_ned
        timestamp = datetime.datetime.now().isoformat()
        pos = position_ned.position
        escribir_csv(ARCHIVO_TRAYECTORIA, [timestamp, pos.north_m, pos.east_m, pos.down_m])
        await asyncio.sleep(0.1)

async def cambiar_estado(nuevo_estado, drone):
    """Cambia estado, loggea, resetea variables y detiene dron si es necesario."""
    global estado_actual, objetivo_perdido_contador, min_distancia_vista, fase_avance, ciclos_sin_objetivo_buscando
    estado_actual_nombre = estado_nombres.get(estado_actual, f"DESCONOCIDO({estado_actual})")
    nuevo_estado_nombre = estado_nombres.get(nuevo_estado, f"DESCONOCIDO({nuevo_estado})")

    if estado_actual != nuevo_estado:
        print(f"Estado: {estado_actual_nombre} -> {nuevo_estado_nombre}")
        estado_actual = nuevo_estado
        objetivo_perdido_contador = 0
        ciclos_sin_objetivo_buscando = 0 # Resetear contador de búsqueda también

        # Resetear rastreo de distancia al empezar a alinear una nueva puerta
        if nuevo_estado == ESTADO_ALINEANDO:
            min_distancia_vista = float('inf')
            fase_avance = 'INICIAL'

        # Detener dron al empezar a buscar (tras pasar una) o al completar misión
        if nuevo_estado in [ESTADO_BUSCANDO, ESTADO_MISION_COMPLETA]:
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(0.2)

async def mover(drone, offset_x, offset_y, distancia, num_targets):
    """Controla movimiento y estados basado en datos recibidos y tendencia de distancia."""
    global estado_actual, objetivo_perdido_contador, min_distancia_vista, fase_avance
    global ciclos_sin_objetivo_buscando, puertas_pasadas
    velocidad_avance, velocidad_lateral, velocidad_vertical = 0.0, 0.0, 0.0
    target_detected = num_targets > 0 and distancia > 0 # Condición para buscar inicialmente

    if estado_actual == ESTADO_BUSCANDO:
        if target_detected:
            print(f"INFO: Objetivo encontrado! (Puerta #{puertas_pasadas + 1})")
            ciclos_sin_objetivo_buscando = 0 # Resetear contador si vemos algo
            await cambiar_estado(ESTADO_ALINEANDO, drone)
        else:
            # Si no detectamos target, incrementamos contador de búsqueda
            ciclos_sin_objetivo_buscando += 1
            print(f"INFO: Buscando siguiente puerta... (Ciclo sin objetivo: {ciclos_sin_objetivo_buscando}/{CONTADOR_BUSQUEDA_MAX})")
            if ciclos_sin_objetivo_buscando >= CONTADOR_BUSQUEDA_MAX:
                print("INFO: No se encontraron más puertas tras búsqueda. Misión completada.")
                await cambiar_estado(ESTADO_MISION_COMPLETA, drone)
            # Mientras buscamos, el dron estará quieto
            #SI QUEREMOS QUE SE MUEVVA EL DRON AL BUSCAR SE PONDRÁ AQUÍ

    elif estado_actual == ESTADO_ALINEANDO:
        ciclos_sin_objetivo_buscando = 0
        if not target_detected: # Si perdemos el target mientras alineamos
            objetivo_perdido_contador += 1
            if objetivo_perdido_contador >= CONTADOR_PERDIDO_MAX:
                print("WARN: P1 perdida alineando, reintentando búsqueda.")
                await cambiar_estado(ESTADO_BUSCANDO, drone)
        else: # Si vemos el target, alineamos
            objetivo_perdido_contador = 0
            alineado_x = abs(offset_x) <= MARGEN_ERROR_X
            alineado_y = abs(offset_y) <= MARGEN_ERROR_Y
            if not alineado_x: velocidad_lateral = -VELOCIDAD_CORRECCION_X if offset_x < 0 else VELOCIDAD_CORRECCION_X
            if not alineado_y: velocidad_vertical = VELOCIDAD_CORRECCION_Y if offset_y > 0 else -VELOCIDAD_CORRECCION_Y
            if alineado_x and alineado_y:
                await cambiar_estado(ESTADO_AVANZANDO, drone)

    elif estado_actual == ESTADO_AVANZANDO:
        ciclos_sin_objetivo_buscando = 0
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
                #Es decir, ahora la distancia a la primera puerta que recibo es X, pero yo estuve a una 
                #distancia menor a esa primera puerta, por lo tanto la he pasado, y la anterior
                # P2 es ahora la nueva P1 (siempre con un margen de error).

                #Aplicar un avance extra para asegurar el paso de la puerta
                
                #AHORA EL AVANCE EXTRA SE APLICA EN if puerta_pasada \/
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
            timestamp = datetime.datetime.now().isoformat()
            if ultima_posicion_ned:
                pos = ultima_posicion_ned.position
                print(f"--- LOG PASO PUERTA {puertas_pasadas + 1}: T={timestamp}, N={pos.north_m:.2f}, E={pos.east_m:.2f}, D={pos.down_m:.2f} ---")
                escribir_csv(ARCHIVO_PASO_PUERTAS, [timestamp, pos.north_m, pos.east_m, pos.down_m])
            else:
                print(f"WARN: No hay datos de posición para loggear paso Puerta {puertas_pasadas + 1}.")

            puertas_pasadas += 1

            # Aplicar avance extra
            print(f"--- Aplicando avance extra de {TIEMPO_AVANCE_EXTRA:.1f} segundos ---")
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(VELOCIDAD_AVANCE, 0.0, 0.0, 0.0)
            )
            await asyncio.sleep(TIEMPO_AVANCE_EXTRA)

            await cambiar_estado(ESTADO_BUSCANDO, drone)
            return 

    elif estado_actual == ESTADO_MISION_COMPLETA:
        pass # Se acabó

    #Y finalmente aplicar ka velocidad calculada en los pasos anteriores
    if estado_actual not in [ESTADO_MISION_COMPLETA, ESTADO_BUSCANDO]:
         # No aplicar velocidad, si estabamos avanzando pero se perdió la puerta
         if not (estado_actual == ESTADO_AVANZANDO and not target_detected):
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(velocidad_avance, velocidad_lateral, velocidad_vertical, 0.0))
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
    global estado_actual, min_distancia_vista, fase_avance, log_task
    global ciclos_sin_objetivo_buscando, puertas_pasadas 
    estado_actual = ESTADO_BUSCANDO
    min_distancia_vista = float('inf')
    fase_avance = 'INICIAL'
    ciclos_sin_objetivo_buscando = 0
    puertas_pasadas = 0 
    log_task = None
    try:
        #Inicializar Archivos CSV 
        print(f"Inicializando archivos CSV: {ARCHIVO_TRAYECTORIA}, {ARCHIVO_PASO_PUERTAS}")
        cabecera = ["Timestamp", "North_m", "East_m", "Down_m"]
        try: 
            with open(ARCHIVO_TRAYECTORIA, mode='w', newline='', encoding='utf-8') as file:
                writer = csv.writer(file)
                writer.writerow(cabecera)
            with open(ARCHIVO_PASO_PUERTAS, mode='w', newline='', encoding='utf-8') as file:
                writer = csv.writer(file)
                writer.writerow(cabecera)
        except Exception as e:
            print(f"FATAL: No se pudieron inicializar los archivos CSV: {e}")
            return
        
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

        global log_task
        log_task = asyncio.create_task(log_trayectoria(drone))

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

        global log_task
        if log_task and not log_task.done():
            log_task.cancel()
            await asyncio.wait_for(log_task, timeout=1.0)


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