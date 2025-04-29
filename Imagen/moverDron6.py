"""
moverDron6.py
Adaptado de moverDron5.py para aterrizaje de precisión en plataforma móvil.
Busca una plataforma (detectada por tratarImagen), se alinea horizontalmente,
desciende manteniendo la alineación y finalmente aterriza.
"""
import asyncio
import socket
import csv
import datetime
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed) # PositionNedYaw no se usa aquí

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
ESTADO_MISION_COMPLETA = 7
CONTADOR_BUSQUEDA_MAX = 50 # Num máx de ciclos buscando sin encontrar antes de terminar

estado_nombres = {
    ESTADO_INICIO: "INICIO",
    ESTADO_BUSCANDO_PLATAFORMA: "BUSCANDO_PLATAFORMA",
    ESTADO_ALINEANDO: "ALINEANDO",
    ESTADO_DESCENDIENDO: "DESCENDIENDO",
    ESTADO_ATERRIZANDO: "ATERRIZANDO",
    ESTADO_MISION_COMPLETA: "COMPLETA"
}

estado_actual = ESTADO_BUSCANDO_PLATAFORMA # Empezamos buscando
objetivo_perdido_contador = 0             # Contador de ciclos consecutivos sin detectar el objetivo actual.
ciclos_sin_objetivo_buscando = 0          # Contador para el timeout en búsqueda.



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
    print("a")

    if estado_actual == ESTADO_BUSCANDO_PLATAFORMA:
        print("a")

    elif estado_actual == ESTADO_ALINEANDO_HORIZONTAL:
        print("a")

    elif estado_actual == ESTADO_DESCENDIENDO:
        print("a")


    elif estado_actual == ESTADO_ATERRIZANDO:
        print("a")

    elif estado_actual == ESTADO_MISION_COMPLETA:
        print("a")



# --- Función Principal de Recepción UDP ---
async def recibir_posiciones(drone, sock):
    """Bucle principal: recibe UDP y llama a mover()."""
    loop = asyncio.get_event_loop()
    global estado_actual
    print(f"--- Estado Inicial: {estado_nombres.get(estado_actual, 'DESCONOCIDO')} ---")

    while estado_actual != ESTADO_MISION_COMPLETA:
        try:
            data, addr = await loop.run_in_executor(None, sock.recvfrom, 1024)
            mensaje = data.decode('utf-8')
            try:
                x_str, y_str, dist_str, num_t_str = mensaje.split(',')
                offset_x, offset_y = float(x_str), float(y_str)
                distancia, num_targets = float(dist_str), int(num_t_str)
                # Llamar a la lógica de movimiento/estado con los datos recibidos
                await mover(drone, offset_x, offset_y, distancia, num_targets)
            except ValueError:
                # print(f"WARN: Mensaje UDP inválido: '{mensaje}'") # Log opcional
                continue # Ignorar mensaje mal formateado
        except asyncio.CancelledError:
             print("WARN: recibir_posiciones cancelado.")
             break
        except Exception as e:
             print(f"ERROR CRÍTICO en recibir_posiciones: {e}. Finalizando misión.")
             await cambiar_estado(ESTADO_MISION_COMPLETA, drone)
             break

    print("--- Bucle principal de recepción finalizado ---")


async def run():
    """Función principal: conecta, prepara, ejecuta misión y logging."""
    drone = System()
    sock = None
    global estado_actual, objetivo_perdido_contador, ciclos_sin_objetivo_buscando

    estado_actual = ESTADO_BUSCANDO_PLATAFORMA
    objetivo_perdido_contador = 0
    ciclos_sin_objetivo_buscando = 0
    log_task = None
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
        await asyncio.sleep(15)

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
        if log_task and not log_task.done():
            log_task.cancel()
            await asyncio.wait_for(log_task, timeout=1.0)

        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        )

        print("-- Stopping offboard")
        try:
            await drone.offboard.stop() 
        except OffboardError as error: 
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