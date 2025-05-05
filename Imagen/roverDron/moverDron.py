"""
moverDron5.py
Adaptado para aterrizaje de precisión en plataforma (roja/gris).
Busca una plataforma, se alinea horizontalmente usando cámara inferior,
desciende manteniendo alineación y aterriza.
"""
import asyncio
import socket
import csv
import datetime
import math
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed)

SERVER_IP = "127.0.0.1"         # Dirección IP donde este script escucha
SERVER_PORT = 65432           
MAVSDK_PORT = 50051


VELOCIDAD_CORRECCION_X = 0.4  # Velocidad LATERAL (Y) por error en X de imagen
VELOCIDAD_CORRECCION_Y = 0.4  # Velocidad AVANCE (X) por error en Y de imagen
VELOCIDAD_DESCENSO = 0.2     

MARGEN_ERROR_X = 5          # Error máx. permitido en píxeles (offset X imagen) para considerar alineado
MARGEN_ERROR_Y = 5           # Error máx. permitido en píxeles (offset Y imagen) para considerar alineado
MARGEN_ERROR_X_ALINEADO = 5
MARGEN_ERROR_Y_ALINEADO = 5

ALTITUD_ATERRIZAJE = 0.4      # Altitud estimada (m) sobre la plataforma para iniciar comando land()

ESTADO_BUSCANDO_PLATAFORMA = 1
ESTADO_ALINEANDO_HORIZONTAL = 2
ESTADO_DESCENDIENDO = 3
ESTADO_ATERRIZANDO = 4
ESTADO_MISION_COMPLETA = 7

estado_nombres = {
    ESTADO_BUSCANDO_PLATAFORMA: "BUSCANDO_PLATAFORMA",
    ESTADO_ALINEANDO_HORIZONTAL: "ALINEANDO_HORIZONTAL",
    ESTADO_DESCENDIENDO: "DESCENDIENDO",
    ESTADO_ATERRIZANDO: "ATERRIZANDO",
    ESTADO_MISION_COMPLETA: "COMPLETA"
}

estado_actual = ESTADO_BUSCANDO_PLATAFORMA

async def cambiar_estado(nuevo_estado, drone):
    """Cambia estado, loggea y detiene dron si es necesario."""
    global estado_actual, objetivo_perdido_contador, ciclos_sin_objetivo_buscando
    estado_actual_nombre = estado_nombres.get(estado_actual, f"DESCONOCIDO({estado_actual})")
    nuevo_estado_nombre = estado_nombres.get(nuevo_estado, f"DESCONOCIDO({nuevo_estado})")

    if estado_actual != nuevo_estado:
        print(f"Estado: {estado_actual_nombre} -> {nuevo_estado_nombre}")
        estado_actual = nuevo_estado

        if nuevo_estado in [ESTADO_BUSCANDO_PLATAFORMA, ESTADO_MISION_COMPLETA]:
             try:
                 await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
                 await asyncio.sleep(0.2)
             except Exception as e:
                  print(f"WARN: Error al detener dron en cambiar_estado: {e}")

async def mover(drone, offset_x, offset_y, distancia, num_targets):
    """Controla el dron para alinearse y descender sobre la plataforma."""
    global estado_actual

    velocidad_lateral = 0.0
    velocidad_vertical = 0.0
    velocidad_altura = 0.0

    target_detected = num_targets > 0 

    if estado_actual == ESTADO_BUSCANDO_PLATAFORMA:
        if target_detected:
            print(f"INFO: Plataforma encontrada!")
            await cambiar_estado(ESTADO_ALINEANDO_HORIZONTAL, drone)


    if estado_actual == ESTADO_ALINEANDO_HORIZONTAL: #Corregimos en ejes X e Y (2D)
        if not target_detected:
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            print("WARN: Plataforma perdida durante alineación horizontal. Volviendo a buscar.")
            await cambiar_estado(ESTADO_BUSCANDO_PLATAFORMA, drone)
        else: 
            #Comprobamos si necesita corrección en eje X;
            if abs(offset_x) > MARGEN_ERROR_X:
                print(f"--- Estado: Alineando X --- Recibido Offset X: {offset_x:.1f}")
                if offset_x > 0:
                    velocidad_lateral = -VELOCIDAD_CORRECCION_X
                    print(f"Moviendo IZQUIERDA ({velocidad_lateral=})")
                else:
                    velocidad_lateral = VELOCIDAD_CORRECCION_X
                    print(f"Moviendo DERECHA ({velocidad_lateral=})")
                #Asignamos VELOCIDAD_CORRECION_X a velocidad_lateral con su correspondiente signo

            #Comprbamos si necesita corrección en eje Y;
            if abs(offset_y) > MARGEN_ERROR_Y:
                print(f"--- Estado: Alineando Y --- Recibido Offset Y: {offset_y:.1f}")
                if offset_y > 0:
                    velocidad_vertical = -VELOCIDAD_CORRECCION_Y
                    print(f"Moviendo ARRIBA ({velocidad_vertical=})")
                else:
                    velocidad_vertical = VELOCIDAD_CORRECCION_Y
                    print(f"Moviendo ABAJO ({velocidad_vertical=})")
                #Asignamos VELOCIDAD_CORRECION_Y a velocidad_vertical con su correspondiente signo
            
            if abs(offset_x) < MARGEN_ERROR_X and abs(offset_y) < MARGEN_ERROR_Y:
                print(f"--- Alineado horizontalmente sobre plataforma (Offsets: X={offset_x:.0f}, Y={offset_y:.0f}). Iniciando descenso. ---")
                await cambiar_estado(ESTADO_DESCENDIENDO, drone)
            else:
                await drone.offboard.set_velocity_body(
                     VelocityBodyYawspeed(velocidad_lateral, velocidad_vertical, velocidad_altura, 0.0)
                )

    elif estado_actual == ESTADO_DESCENDIENDO:
        if target_detected:
            if abs(offset_x) > MARGEN_ERROR_X_ALINEADO:
                print(f"--- Estado: Alineando X --- Recibido Offset X: {offset_x:.1f}")
                if offset_x > 0:
                    velocidad_lateral = -VELOCIDAD_CORRECCION_X
                    print(f"Moviendo IZQUIERDA ({velocidad_lateral=})")
                else:
                    velocidad_lateral = VELOCIDAD_CORRECCION_X
                    print(f"Moviendo DERECHA ({velocidad_lateral=})")
                #Asignamos VELOCIDAD_CORRECION_X a velocidad_lateral con su correspondiente signo

            #Comprbamos si necesita corrección en eje Y;
            if abs(offset_y) > MARGEN_ERROR_Y_ALINEADO:
                print(f"--- Estado: Alineando Y --- Recibido Offset Y: {offset_y:.1f}")
                if offset_y > 0:
                    velocidad_vertical = -VELOCIDAD_CORRECCION_Y
                    print(f"Moviendo ARRIBA ({velocidad_vertical=})")
                else:
                    velocidad_vertical = VELOCIDAD_CORRECCION_Y
                    print(f"Moviendo ABAJO ({velocidad_vertical=})")
                #Asignamos VELOCIDAD_CORRECION_Y a velocidad_vertical con su correspondiente signo

            velocidad_altura = VELOCIDAD_DESCENSO

            if distancia < ALTITUD_ATERRIZAJE:
                print(f"INFO: Altitud ({distancia:.2f}m) por debajo de umbral ({ALTITUD_ATERRIZAJE}m). Comandando LAND.")
                await cambiar_estado(ESTADO_ATERRIZANDO, drone)
                return
            else:
                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(velocidad_lateral, velocidad_vertical, velocidad_altura, 0.0)
                )
        else:
            print(f"WARN: Objetivo perdido inesperadamente durante descenso! Deteniendo.")
            await cambiar_estado(ESTADO_ALINEANDO_HORIZONTAL, drone)

    elif estado_actual == ESTADO_ATERRIZANDO:
        print("--- Comandando drone.action.land() ---")
        try:
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(0.2)
            await drone.action.land()
            print("--- Comando Land enviado. Finalizando lógica del script. ---")
            await cambiar_estado(ESTADO_MISION_COMPLETA, drone)
        except Exception as e:
            print(f"ERROR al intentar comandar land(): {e}")
            await cambiar_estado(ESTADO_MISION_COMPLETA, drone)
        return

    elif estado_actual == ESTADO_MISION_COMPLETA:
        pass


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
                await mover(drone, offset_x, offset_y, distancia, num_targets)
            except ValueError:
                # print(f"WARN: Mensaje UDP inválido: '{mensaje}'") # Log opcional
                continue
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
    drone = System(mavsdk_server_address="localhost", port=MAVSDK_PORT)
    sock = None
    global estado_actual

    estado_actual = ESTADO_BUSCANDO_PLATAFORMA  

    try:
        print("Conectando al dron...")
        await drone.connect()
        async for state in drone.core.connection_state():
            if state.is_connected: print("-- Conectado al dron!"); break
            await asyncio.sleep(0.1)

        print("Esperando posición global del dron...")
        async for health in drone.telemetry.health():
            # Ajustar si es necesario, quizás solo is_local_position_ok baste
            if health.is_global_position_ok: print("-- GPS OK."); break
            print(f"-- Esperando GPS OK (Actual: {health.is_global_position_ok})...")

        # Configurar Socket UDP
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((SERVER_IP, SERVER_PORT))
        print(f"-- Socket UDP escuchando en {SERVER_IP}:{SERVER_PORT}")

        # Secuencia de Vuelo
        print("-- Armando dron")
        await drone.action.arm()
        print("-- Estableciendo setpoint inicial")
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        print("-- Iniciando modo offboard")
        try:
            await drone.offboard.start()
        except OffboardError as error:
            print(f"FATAL: Error al iniciar modo offboard: {error}. Desarmando.")
            try: await drone.action.disarm()
            except Exception: pass
            if sock: sock.close()
            return

        print("-- Subo")
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, -2.0, 0.0))
        await asyncio.sleep(12)

        await recibir_posiciones(drone, sock)

        print("--- Bucle principal terminado. Iniciando secuencia final. ---")

    except asyncio.CancelledError:
        print("WARN: Tarea principal (run) cancelada.")
    except Exception as e:
        print(f"ERROR INESPERADO en run(): {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("--- Ejecutando bloque Finally: Deteniendo y Limpiando ---")
        try:
             await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
             await asyncio.sleep(0.5)
        except Exception: pass

        print("-- Stopping offboard")
        try:
            await drone.offboard.stop()
        except Exception: pass

        print("-- Intentando desarmar...")
        try:
            await drone.action.disarm()
            print("-- Desarmado final comandado.")
        except Exception as e_disarm:
            print(f"WARN: Fallo en disarm final: {e_disarm}")

        if sock: sock.close(); print("-- Socket UDP cerrado.")
        print("--- Secuencia de finalización completada. ---")

if __name__ == "__main__":
    print("Iniciando script de control (Aterrizaje en Plataforma)...")
    asyncio.run(run())
    print("Script de control (Aterrizaje en Plataforma) finalizado.")