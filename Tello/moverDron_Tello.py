import asyncio
import socket
import csv
import datetime
import time
from djitellopy import Tello
import threading
from pynput import keyboard

SERVER_IP = "127.0.0.1"
SERVER_PORT = 65432
ARCHIVO_PASO_PUERTAS = "paso_puertas_tello.csv"
VELOCIDAD_AVANCE_TELLO = 35
VELOCIDAD_LATERAL_TELLO = 40
VELOCIDAD_VERTICAL_TELLO = 45
VELOCIDAD_YAW_TELLO = 0

MARGEN_ERROR_X = 30
MARGEN_ERROR_Y = 30
MARGEN_ERROR_X_ALINEADO = 40
MARGEN_ERROR_Y_ALINEADO = 40

CONTADOR_PERDIDO_MAX = 15
DISTANCIA_UMBRAL_CERCA = 1.5
UMBRAL_AUMENTO_DIST = 0.5
TIEMPO_AVANCE_EXTRA = 1.5
CONTADOR_BUSQUEDA_MAX = 60

ESTADO_INICIO = 0
ESTADO_BUSCANDO = 1
ESTADO_ALINEANDO = 2
ESTADO_AVANZANDO = 3
ESTADO_ATERRIZANDO = 6 
ESTADO_MISION_COMPLETA = 7
estado_nombres = {
    ESTADO_INICIO: "INICIO", ESTADO_BUSCANDO: "BUSCANDO_PUERTA",
    ESTADO_ALINEANDO: "ALINEANDO", ESTADO_AVANZANDO: "AVANZANDO",
    ESTADO_ATERRIZANDO: "ATERRIZANDO",
    ESTADO_MISION_COMPLETA: "COMPLETA"
}

estado_actual = ESTADO_INICIO
objetivo_perdido_contador = 0
min_distancia_vista = float('inf')
fase_avance = 'INICIAL'
puertas_pasadas = 0
ciclos_sin_objetivo_buscando = 0
parada_solicitada = asyncio.Event() 
aterrizaje_forzado = False # Para saber si fue 'q'
emergencia_activada = False # Para saber si fue 'w'

def escribir_csv(filename, data_row):
    with open(filename, mode='a', newline='', encoding='utf-8') as file_object:
        writer = csv.writer(file_object)
        writer.writerow(data_row)

async def cambiar_estado(nuevo_estado, tello):
    global estado_actual, objetivo_perdido_contador, min_distancia_vista, fase_avance, ciclos_sin_objetivo_buscando
    estado_actual_nombre = estado_nombres.get(estado_actual, f"DESCONOCIDO({estado_actual})")
    nuevo_estado_nombre = estado_nombres.get(nuevo_estado, f"DESCONOCIDO({nuevo_estado})")

    if estado_actual != nuevo_estado:
        print(f"Estado: {estado_actual_nombre} -> {nuevo_estado_nombre}")
        estado_actual = nuevo_estado
        objetivo_perdido_contador = 0
        ciclos_sin_objetivo_buscando = 0

        if nuevo_estado == ESTADO_ALINEANDO:
            min_distancia_vista = float('inf')
            fase_avance = 'INICIAL'

        if nuevo_estado in [ESTADO_BUSCANDO, ESTADO_MISION_COMPLETA, ESTADO_ATERRIZANDO]:
             if not emergencia_activada: 
                print("Deteniendo movimiento...")
                try:
                    tello.send_rc_control(0, 0, 0, 0)
                    await asyncio.sleep(0.1)
                except Exception as e:
                    print(f"Advertencia: No se pudo enviar comando de parada: {e}")


async def mover(tello, offset_x, offset_y, distancia, num_targets):
    global estado_actual, objetivo_perdido_contador, min_distancia_vista, fase_avance
    global ciclos_sin_objetivo_buscando, puertas_pasadas, emergencia_activada

    if emergencia_activada:
        return
    lr, fb, ud, yv = 0, 0, 0, 0
    target_detected = num_targets > 0 and distancia > 0 and distancia != float('inf')

    if estado_actual == ESTADO_BUSCANDO:
        if target_detected:
            print(f"INFO: ¡Objetivo encontrado! (Puerta #{puertas_pasadas + 1}) Dist: {distancia:.2f}m")
            ciclos_sin_objetivo_buscando = 0
            await cambiar_estado(ESTADO_ALINEANDO, tello)
        else:
            ciclos_sin_objetivo_buscando += 1
            print(f"INFO: Buscando siguiente puerta... (Ciclo sin objetivo: {ciclos_sin_objetivo_buscando}/{CONTADOR_BUSQUEDA_MAX})")
            if ciclos_sin_objetivo_buscando >= CONTADOR_BUSQUEDA_MAX:
                print("INFO: No se encontraron más puertas tras búsqueda exhaustiva. Misión completada.")
                await cambiar_estado(ESTADO_MISION_COMPLETA, tello)
            lr, fb, ud, yv = 0, 0, 0, 0

    elif estado_actual == ESTADO_ALINEANDO:
        ciclos_sin_objetivo_buscando = 0 # Resetear contador de búsqueda si estamos alineando

        if not target_detected:
            objetivo_perdido_contador += 1
            print(f"WARN: Objetivo perdido durante alineación ({objetivo_perdido_contador}/{CONTADOR_PERDIDO_MAX})")
            if objetivo_perdido_contador >= CONTADOR_PERDIDO_MAX:
                print("WARN: Objetivo perdido definitivamente. Volviendo a buscar.")
                await cambiar_estado(ESTADO_BUSCANDO, tello)
            lr, fb, ud, yv = 0, 0, 0, 0
        else:
            objetivo_perdido_contador = 0 
            alineado_x = abs(offset_x) <= MARGEN_ERROR_X
            alineado_y = abs(offset_y) <= MARGEN_ERROR_Y

            # Calcular velocidad lateral (izquierda/derecha) si no está alineado en X
            if not alineado_x:
                # Si offset_x < 0 (objetivo a la izquierda), mover dron a la izquierda (lr negativo)
                # Si offset_x > 0 (objetivo a la derecha), mover dron a la derecha (lr positivo)
                lr = -VELOCIDAD_LATERAL_TELLO if offset_x < 0 else VELOCIDAD_LATERAL_TELLO

            # Calcular velocidad vertical (arriba/abajo) si no está alineado en Y
            if not alineado_y:
                # Si offset_y > 0 (objetivo ABAJO en imagen), mover dron HACIA ARRIBA (ud positivo)
                # Si offset_y < 0 (objetivo ARRIBA en imagen), mover dron HACIA ABAJO (ud negativo)
                ud = VELOCIDAD_VERTICAL_TELLO if offset_y > 0 else -VELOCIDAD_VERTICAL_TELLO

            # Si está alineado en ambos ejes, cambiar a estado AVANZANDO
            if alineado_x and alineado_y:
                print("INFO: Alineado con la puerta. Iniciando avance.")
                await cambiar_estado(ESTADO_AVANZANDO, tello)
                lr, fb, ud, yv = 0, 0, 0, 0

    elif estado_actual == ESTADO_AVANZANDO:
        ciclos_sin_objetivo_buscando = 0 
        puerta_pasada = False 

        if target_detected:
            objetivo_perdido_contador = 0

            if distancia < float('inf'):
                min_distancia_vista = min(min_distancia_vista, distancia)

            if distancia < DISTANCIA_UMBRAL_CERCA:
                if fase_avance == 'INICIAL':
                    print(f"--- Fase Avance: CERCA (Dist: {distancia:.2f}m < {DISTANCIA_UMBRAL_CERCA}m) ---")
                fase_avance = 'CERCA'

            # Condición de Paso 1: Aumento significativo de distancia después de haber estado cerca
            # Mismas lógicas que para el de px4
            if fase_avance == 'CERCA' and distancia > min_distancia_vista + UMBRAL_AUMENTO_DIST:
                print(f"INFO: Paso detectado (Cond 1: Aumento dist: {distancia:.2f}m > min {min_distancia_vista:.2f}m + {UMBRAL_AUMENTO_DIST}m)")
                puerta_pasada = True

            # Si NO se ha detectado el paso todavía, seguir avanzando y corrigiendo
            if not puerta_pasada:
                fb = VELOCIDAD_AVANCE_TELLO 

                # Corrección lateral si nos desviamos del margen ampliado
                if abs(offset_x) > MARGEN_ERROR_X_ALINEADO:
                    lr = -VELOCIDAD_LATERAL_TELLO if offset_x < 0 else VELOCIDAD_LATERAL_TELLO
                # Corrección vertical si nos desviamos del margen ampliado
                if abs(offset_y) > MARGEN_ERROR_Y_ALINEADO:
                    ud = VELOCIDAD_VERTICAL_TELLO if offset_y > 0 else -VELOCIDAD_VERTICAL_TELLO

        else:
            # Objetivo NO detectado mientras avanzábamos
            objetivo_perdido_contador += 1
            print(f"INFO: Objetivo no detectado avanzando ({objetivo_perdido_contador}/{CONTADOR_PERDIDO_MAX}). Comprobando posible paso...")
            # Detenerse mientras se confirma la pérdida
            lr, fb, ud, yv = 0, 0, 0, 0

            # Si se supera el contador de pérdida
            if objetivo_perdido_contador >= CONTADOR_PERDIDO_MAX:
                # Condición de Paso 2: Pérdida confirmada. Comprobar si habíamos estado CERCA.
                if fase_avance == 'CERCA':
                    print("INFO: Paso confirmado (Cond 2: Pérdida de objetivo tras estar cerca).")
                    puerta_pasada = True
                else:
                    print("WARN: Objetivo perdido antes de confirmar cercanía. Asumiendo paso igualmente.")
                    puerta_pasada = True

        if puerta_pasada:
            puertas_pasadas += 1
            timestamp = datetime.datetime.now().isoformat()

            try:
                altura_actual = tello.get_height()
                bateria_actual = tello.get_battery()
                print(f"--- LOG PASO PUERTA {puertas_pasadas}: T={timestamp}, Altura={altura_actual}cm, Bat={bateria_actual}% ---")
                escribir_csv(ARCHIVO_PASO_PUERTAS, [timestamp, puertas_pasadas, altura_actual, bateria_actual])
            except Exception as log_err:
                print(f"WARN: No se pudieron obtener datos extra para loggear paso Puerta {puertas_pasadas}: {log_err}")
                escribir_csv(ARCHIVO_PASO_PUERTAS, [timestamp, puertas_pasadas, "N/A", "N/A"])

            print(f"--- Aplicando avance extra de {TIEMPO_AVANCE_EXTRA:.1f} segundos ---")
            try:
                tello.send_rc_control(0, VELOCIDAD_AVANCE_TELLO, 0, 0)
                await asyncio.sleep(TIEMPO_AVANCE_EXTRA)
                tello.send_rc_control(0, 0, 0, 0)
                print("--- Avance extra completado ---")
            except Exception as e:
                print(f"Warn: Error durante el avance extra: {e}")
                try: tello.send_rc_control(0, 0, 0, 0)
                except: pass 

            await cambiar_estado(ESTADO_BUSCANDO, tello)
            return

    elif estado_actual == ESTADO_ATERRIZANDO:
        lr, fb, ud, yv = 0, 0, 0, 0
        print("INFO: En estado de aterrizaje, esperando finalización...")

    elif estado_actual == ESTADO_MISION_COMPLETA:
        lr, fb, ud, yv = 0, 0, 0, 0
        print("INFO: Misión completa, sin movimiento.")

    if estado_actual not in [ESTADO_MISION_COMPLETA, ESTADO_ATERRIZANDO]:
        if not (estado_actual == ESTADO_BUSCANDO and lr==0 and fb==0 and ud==0 and yv==0) and \
           not (estado_actual == ESTADO_AVANZANDO and not target_detected):
            tello.send_rc_control(lr, fb, ud, yv)


async def recibir_posiciones(tello, sock):
    loop = asyncio.get_event_loop()
    global estado_actual, parada_solicitada, emergencia_activada

    print(f"--- Estado Inicial: {estado_nombres[estado_actual]} ---")
    print("--- LISTO para recibir datos UDP. Pulsa 'q' para aterrizar, 'w' para EMERGENCIA ---")

    while estado_actual not in [ESTADO_MISION_COMPLETA, ESTADO_ATERRIZANDO] and not emergencia_activada:
        if parada_solicitada.is_set():
            print("--> Señal de parada recibida (tecla 'q'). Iniciando aterrizaje...")
            global aterrizaje_forzado
            aterrizaje_forzado = True
            await cambiar_estado(ESTADO_ATERRIZANDO, tello)
            break

        try:
            data, addr = await loop.run_in_executor(None, sock.recvfrom, 1024)
            mensaje = data.decode('utf-8')
            try:
                x_str, y_str, dist_str, num_t_str = mensaje.split(',')
                offset_x, offset_y = float(x_str), float(y_str)
                distancia, num_targets = float(dist_str), int(num_t_str)

                if estado_actual not in [ESTADO_ATERRIZANDO, ESTADO_MISION_COMPLETA]:
                     await mover(tello, offset_x, offset_y, distancia, num_targets)

            except ValueError:
                print(f"WARN: Mensaje UDP inválido: '{mensaje}'")
                continue
            except Exception as move_err:
                print(f"ERROR en mover(): {move_err}")
                await cambiar_estado(ESTADO_MISION_COMPLETA, tello)
                break

        except ConnectionResetError:
             print("ERROR: El cliente UDP (procesamiento de imagen) se desconectó.")
             await cambiar_estado(ESTADO_ATERRIZANDO, tello) 
             break
        except socket.timeout: 
             continue
        except Exception as e:
            print(f"ERROR en recibir_posiciones: {e}. Finalizando misión.")
            await cambiar_estado(ESTADO_ATERRIZANDO, tello) 
            break
        await asyncio.sleep(0.01)

    print("--- Bucle principal de recepción finalizado ---")


def monitor_teclado(tello_inst, stop_event_param):
    global emergencia_activada 
    print("[Hilo Teclado] Iniciado. Escuchando 'q' y 'w'...")

    def on_press(key):
        global emergencia_activada 
        if key.char == 'q':
            if not stop_event_param.is_set() and not emergencia_activada:
                print("[Hilo Teclado] 'q' presionada. Solicitando parada y aterrizaje...")
                stop_event_param.set()
        elif key.char == 'w':
                if not emergencia_activada:
                    print("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                    print("[Hilo Teclado] 'w' presionada. ¡¡¡PARADA DE EMERGENCIA!!!")
                    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                    emergencia_activada = True # Poner la bandera global
                    tello_inst.emergency() # ¡Corta motores inmediatamente!


    with keyboard.Listener(on_press=on_press) as listener:
        try:
            listener.join() 
        except Exception as e:
             print(f"[Hilo Teclado] Error en listener.join: {e}")
    print("[Hilo Teclado] Finalizado.")


async def run():
    tello = Tello()
    sock = None
    global estado_actual, min_distancia_vista, fase_avance, parada_solicitada
    global ciclos_sin_objetivo_buscando, puertas_pasadas, aterrizaje_forzado, emergencia_activada

    estado_actual = ESTADO_INICIO
    min_distancia_vista = float('inf')
    fase_avance = 'INICIAL'
    ciclos_sin_objetivo_buscando = 0
    puertas_pasadas = 0
    aterrizaje_forzado = False
    emergencia_activada = False
    parada_solicitada.clear()

    keyboard_thread = None

    try:
        print(f"Inicializando archivo CSV: {ARCHIVO_PASO_PUERTAS}")
        cabecera = ["Timestamp", "NumeroPuerta", "Altura_cm", "Bateria_porc"]
        with open(ARCHIVO_PASO_PUERTAS, mode='w', newline='', encoding='utf-8') as file:
            writer = csv.writer(file)
            writer.writerow(cabecera)

        print("Conectando al Tello...")
        tello.connect()
        print(f"Conectado. Batería: {tello.get_battery()}%")

        if tello.get_battery() < 20:
             print("¡¡¡BATERÍA BAJA!!! No se despegará.")
             return

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((SERVER_IP, SERVER_PORT))
        print(f"Escuchando datos de imagen en UDP {SERVER_IP}:{SERVER_PORT}")

        keyboard_thread = threading.Thread(target=monitor_teclado, args=(tello, parada_solicitada), daemon=True)
        keyboard_thread.start()

        print("-- Despegando...")
        try:
             tello.takeoff()
             await asyncio.sleep(5) 
        except Exception as takeoff_err:
             print(f"ERROR al despegar: {takeoff_err}. Abortando.")
             return 

        print("-- Iniciando búsqueda de puertas...")
        await cambiar_estado(ESTADO_BUSCANDO, tello)

        await recibir_posiciones(tello, sock)

        print("--- Saliendo del bucle principal ---")
        if estado_actual == ESTADO_MISION_COMPLETA:
             print("Misión completada con éxito.")
        elif aterrizaje_forzado:
             print("Aterrizaje iniciado por el usuario (tecla 'q').")

    except KeyboardInterrupt:
        print("\nCtrl+C detectado. Solicitando aterrizaje...")
        parada_solicitada.set()
        aterrizaje_forzado = True
    except Exception as e:
        print(f"\nError inesperado durante la ejecución: {e}")
        print("Intentando aterrizar por seguridad...")
        parada_solicitada.set() 
        aterrizaje_forzado = True
    finally:
        print("-- Ejecutando bloque Finally: Limpiando...")

        if emergencia_activada:
             print("-- EMERGENCIA FUE ACTIVADA. No se intentará aterrizaje normal. --")
        else:
            print("-- Intentando aterrizaje final...")
            try:
                tello.send_rc_control(0, 0, 0, 0)
                await asyncio.sleep(0.5)
                tello.land()
                print("-- Comando de aterrizaje enviado. Esperando...")
                await asyncio.sleep(5)
            except Exception as land_err:
                print(f"Error durante el aterrizaje final o ya estaba inactivo: {land_err}")

        if sock:
            sock.close()
            print("-- Socket UDP cerrado.")

        try:
             print("-- Desconectando del Tello...")
             tello.end()
             print("-- Conexión con Tello cerrada.")
        except Exception as end_err:
             print(f"Error cerrando conexión Tello: {end_err}")

        print("-- Secuencia de finalización completada.")

if __name__ == "__main__":
    print("Iniciando script de control para Tello con control de teclado...")
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
         print("\nPrograma interrumpido por el usuario (Ctrl+C en run).")
    except Exception as main_err:
        print(f"\nError fatal en la ejecución principal: {main_err}")
    finally:
        print("Script de control finalizado.")