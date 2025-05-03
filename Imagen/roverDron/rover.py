import asyncio
import math
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

MAVSDK_PORT = 50040

LARGO_PISTA = 12.0  # Metros (longitud de las rectas)
ANCHO_PISTA = 5.0   # Metros (distancia entre las rectas, = 2 * Radio)
ALTITUD_ROVER = -0.5 # Metros NED (negativo para estar sobre el origen Z=0)
VELOCIDAD_MOVIMIENTO = 1.5 # m/s - Velocidad media deseada entre waypoints

# P0: Inicio Recta 1
# P1: Fin Recta 1 / Inicio Curva 1
# P2: Fin Curva 1 / Inicio Recta 2
# P3: Fin Recta 2 / Inicio Curva 2
# P4: Fin Curva 2 (Vuelta a P0)
waypoints = [
    (0.0,           0.0),           # P0: Origen
    (LARGO_PISTA,   0.0),           # P1: Avanza L metros Norte
    (LARGO_PISTA,   ANCHO_PISTA),   # P2: Avanza A metros Este
    (0.0,           ANCHO_PISTA),   # P3: Retrocede L metros Sur (Norte=0)
]

async def run():
    """Conecta, arma y mueve un rover en bucle por una pista."""
    rover = System(mavsdk_server_address="localhost", port=MAVSDK_PORT)

    await rover.connect()
    print(f"Rover conectando en puerto {MAVSDK_PORT}...")
    async for state in rover.core.connection_state():
        if state.is_connected:
            print(f"Rover conectado en puerto {MAVSDK_PORT}")
            break

    print("Esperando posición global del dron...") 
    async for health in rover.telemetry.health():
        if health.is_global_position_ok: 
            print("-- Posición global OK")
            break
        await asyncio.sleep(1)

    print("-- Armando rover")
    await rover.action.arm()

    print("-- Configurando punto inicial y entrando en Offboard...")
    try:
        await rover.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, ALTITUD_ROVER, 0.0))
        await asyncio.sleep(1) 
        await rover.offboard.start()
        print("-- Modo Offboard activado")
    except OffboardError as e:
        print(f"Error al iniciar Offboard: {e}")
        print("-- Desarmando")
        await rover.action.disarm()
        return

    current_waypoint_index = 0
    pos_anterior_n = 0.0
    pos_anterior_e = 0.0

    while True: 
        target_waypoint = waypoints[current_waypoint_index]
        target_n = target_waypoint[0]
        target_e = target_waypoint[1]

        delta_n = target_n - pos_anterior_n
        delta_e = target_e - pos_anterior_e
        yaw_rad = math.atan2(delta_e, delta_n)
        yaw_deg = math.degrees(yaw_rad)

        print(f"Moviéndose a waypoint {current_waypoint_index}: N={target_n:.1f}, E={target_e:.1f} (Yaw={yaw_deg:.0f} deg)")

        await rover.offboard.set_position_ned(
            PositionNedYaw(target_n, target_e, ALTITUD_ROVER, yaw_deg)
        )

        distancia_segmento = math.sqrt(delta_n**2 + delta_e**2)
        if distancia_segmento > 0.1: 
            tiempo_espera = distancia_segmento / VELOCIDAD_MOVIMIENTO
            print(f"  Distancia: {distancia_segmento:.1f}m, Esperando aprox. {tiempo_espera:.1f}s")
            await asyncio.sleep(tiempo_espera)
        else:
            await asyncio.sleep(1.0) 

        pos_anterior_n = target_n
        pos_anterior_e = target_e

        current_waypoint_index = (current_waypoint_index + 1) % len(waypoints)

if __name__ == "__main__":
    print("Iniciando simulación de rover único en pista...")
    main_task = None
    try:
        main_task = asyncio.ensure_future(run()) 
        asyncio.get_event_loop().run_until_complete(main_task)
    except KeyboardInterrupt:
        print("\nCtrl+C detectado. Deteniendo y aterrizando...")
        if main_task:
            main_task.cancel()
            try:
                 asyncio.get_event_loop().run_until_complete(main_task)
            except asyncio.CancelledError:
                 pass 
