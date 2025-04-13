import asyncio
import socket
import numpy as np # Necesario para np.clip
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed, PositionNedYaw)

SERVER_IP = "127.0.0.1"
SERVER_PORT = 65432

KP_X = 0.008  # Ganancia proporcional para el eje X
KP_Y = 0.008  # Ganancia proporcional para el eje Y 

MAX_VEL_LATERAL = 0.5  # Velocidad lateral máxima (m/s)
MAX_VEL_VERTICAL = 0.5   # Velocidad vertical máxima

# Umbrales para considerar que está "suficientemente alineado" para avanzar
UMBRAL_AVANCE_X = 50    # Pixels de error X máximos para empezar a avanzar
UMBRAL_AVANCE_Y = 50    # Pixels de error Y máximos para empezar a avanzar
VELOCIDAD_AVANCE = 0.4  # Velocidad constante hacia adelante cuando está alineado

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

    #Para que nunca se supere la max velocidad lateral o vertical
    vel_lateral = np.clip(vel_lateral, -MAX_VEL_LATERAL, MAX_VEL_LATERAL)
    vel_vertical = np.clip(vel_vertical, -MAX_VEL_VERTICAL, MAX_VEL_VERTICAL)

    # Solo avanzamos si ambos errores X e Y están por debajo de sus umbrales
    if abs(x) < UMBRAL_AVANCE_X and abs(y) < UMBRAL_AVANCE_Y:
        vel_forward = VELOCIDAD_AVANCE
        print(f"    Alineado (|x|<{UMBRAL_AVANCE_X}, |y|<{UMBRAL_AVANCE_Y}). Avanzando.")
    else:
        vel_forward = 0.0
        print(f"    Alineando (|x| o |y| fuera de umbral). No avanza.")

    print(f"    Enviando Velocidad Cuerpo: Adelante={vel_forward:.2f}, Derecha={vel_lateral:.2f}, Abajo={vel_vertical:.2f}")
    try:
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(vel_forward, vel_lateral, vel_vertical, 0.0) # Yaw Speed = 0.0 (sin giro)
        )
    except OffboardError as error:
         print(f"Error al enviar comando de velocidad offboard: {error._result.result}")



async def recibir_posiciones(drone, sock):
    loop = asyncio.get_event_loop()
    print("-- Iniciando bucle de recepción de posiciones UDP...")
    while True:
        try:
            # Esperar datos del socket de forma asíncrona
            data, addr = await loop.run_in_executor(None, sock.recvfrom, 1024)
            mensaje = data.decode('utf-8')
            # print(f"Recibido UDP: {mensaje} desde {addr}") # Descomentar para debug detallado

            # Parsear el mensaje (asume formato "x,y")
            try:
                x_str, y_str = mensaje.split(',')
                x = float(x_str)
                y = float(y_str)

                # Llamar a la nueva función de movimiento proporcional
                await mover_proporcional(drone, x, y)

            except ValueError:
                print(f"Error: Mensaje UDP mal formateado recibido: '{mensaje}'")
            except Exception as parse_err:
                 print(f"Error procesando mensaje '{mensaje}': {parse_err}")

        except BlockingIOError:
            # Esto es normal si no llegan datos y el socket no es bloqueante
            # Permite que otras tareas de asyncio se ejecuten
            await asyncio.sleep(0.02) # Pequeña pausa para ceder control
            continue
        except Exception as e:
            print(f"Error grave en el bucle de recepción UDP: {e}")
            # Aquí podrías decidir detener el dron o tomar otra acción de seguridad
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(1) # Pausa antes de reintentar o salir

async def run():
    """ Función principal que inicializa el dron, el socket y lanza el bucle """
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Esperando conexión con el dron...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Conectado al dron!")
            break
    else: # Se ejecuta si el bucle termina sin break (nunca se conectó)
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

    sock = None # Inicializar a None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(False) # Importante para no bloquear el bucle asyncio
        sock.bind((SERVER_IP, SERVER_PORT))
        print(f"-- Socket UDP escuchando en {SERVER_IP}:{SERVER_PORT}")
    except Exception as sock_err:
        print(f"Error al crear o bindear el socket UDP: {sock_err}")
        return # Salir si no se puede crear el socket

    print("-- Armando dron")
    await drone.action.arm()
    await asyncio.sleep(1) # Pequeña pausa tras armar

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
        if sock: sock.close() # Cerrar socket si se creó
        return

    print("-- Despegando...")
    await drone.action.takeoff()
    await asyncio.sleep(5)



    try:
        await recibir_posiciones(drone, sock)
    except asyncio.CancelledError:
         print("Bucle de recepción cancelado.")
    finally:
        print("-- Finalizando movimientos y deteniendo offboard...")
        try:
             # Detener cualquier movimiento remanente
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(1)
            await drone.offboard.stop()
            print("-- Modo offboard detenido.")
        except OffboardError as error:
            print(f"Error al detener modo offboard: {error._result.result}")
            # Aún así intentar aterrizar

        print(f"-- Dron aterrizando...")
        await drone.action.land()
        await asyncio.sleep(5) # Esperar a que aterrice completamente

        print("-- Desarmando.")
        await drone.action.disarm()

        if sock:
            sock.close()
            print("-- Socket UDP cerrado.")

# --- Punto de Entrada ---
if __name__ == "__main__":
    print("Iniciando script de control de dron proporcional...")
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        print("Script interrumpido por el usuario (Ctrl+C).")
    print("Script finalizado.")




"""
+-------------------------+
| Inicio del Script (run) |
+-------------------------+
            |
            V
+-------------------------+
| Inicializar Conexión    |
| MAVSDK (Drone)          |
+-------------------------+
            |
            V
+-------------------------+
| Realizar Checks         |
| Armar, Iniciar Offboard |
| (Despegar)              |
+-------------------------+
            |
            V
+-------------------------+
| Inicializar Socket UDP  |
+-------------------------+
            |
            V
+-----------------------------------+      <---------------------------------------+
| Bucle `recibir_posiciones` activo |                                              |
+-----------------------------------+                                              |
            |                                                                      |
            V                                                                      |
+-------------------------+                                                        |
| ¿Paquete UDP recibido?  | -- No --> [Esperar Paquete UDP (await recvfrom)] ------+
+-------------------------+             (Cede control a asyncio)                   |
            | Sí                                                                   |
            V                                                                      |
+-------------------------+                                                        |
| Parsear Paquete (x, y)  |                                                        |
+-------------------------+                                                        |
            |                                                                      |
            V                                                                      |
+-------------------------------------+                                            |
| Llamar `mover_proporcional(x, y)`   |                                            |
| (await)                             |                                            |
+-------------------------------------+                                            |
            |                                                                      |
            V (Dentro de mover_p)                                                  |
+-------------------------------------+                                            |
| Calcular vel_forward, vel_lateral,  |                                            |
| vel_vertical (con KP, clip, if/else)|                                            |
+-------------------------------------+                                            |
            |                                                                      |
            V (Dentro de mover_p)                                                  |
+-------------------------------------+                                            |
| Enviar Comando Velocidad            |                                            |
| (await set_velocity_body)           |                                            |
+-------------------------------------+                                            |
            |                                                                      |
            V (Dentro de mover_p)                                                  |
+-------------------------------------+                                            |
| (MAVSDK envía a PX4)                |                                            |
+-------------------------------------+                                            |
            |                                                                      |
            V (Dentro de mover_p)                                                  |
+-------------------------------------+                                            |
| `set_velocity_body` completa        |                                            |
+-------------------------------------+                                            |
            |                                                                      |
            V                                                                      |
+-------------------------------------+                                            |
| `mover_proporcional` completa       |                                            |
+-------------------------------------+                                            |
            |                                                                      |
            +----------------------------------------------------------------------+


(Flujo de Interrupción, ej: Ctrl+C)

+-------------------------+
| Interrupción Detectada  |
| (ej. KeyboardInterrupt/ |
| CancelledError)         |
+-------------------------+
            |
            V
+-------------------------+
| Ejecutar Bloque `finally`|
| (Detener Offboard, Land,|
| Disarm, Cerrar Socket)  |
+-------------------------+
            |
            V
+-------------------------+
| Fin del Script          |
+-------------------------+

"""
