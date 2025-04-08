import asyncio
import socket
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError

SERVER_IP = "127.0.0.1"
SERVER_PORT = 65432

async def follow_leader():
    rover = System(mavsdk_server_address="localhost", port=50041)
    await rover.connect()

    print("🔌 Conectando rover seguidor...")
    async for state in rover.core.connection_state():
        if state.is_connected:
            print("✅ Rover conectado.")
            break

    await rover.action.arm()
    print("🛡️ Rover armado.")

    await rover.action.takeoff()
    print("🚁 Despegando...")
    await asyncio.sleep(5)

    # Posición inicial del seguidor
    async for pos in rover.telemetry.position_velocity_ned():
        follower_start_x = pos.position.north_m
        follower_start_y = pos.position.east_m
        break

    print(f"📌 Posición inicial del seguidor: ({follower_start_x:.2f}, {follower_start_y:.2f})")

    # Crear socket y recibir primera posición del líder
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    sock.bind((SERVER_IP, SERVER_PORT))
    loop = asyncio.get_running_loop()

    leader_start_x = None
    leader_start_y = None
    last_position = {"x": follower_start_x, "y": follower_start_y}  # Inicialmente quieto

    # Enviar posición inicial varias veces para Offboard
    for _ in range(10):
        await rover.offboard.set_position_ned(PositionNedYaw(
            follower_start_x,
            follower_start_y,
            -10.0,
            0.0
        ))
        await asyncio.sleep(0.1)

    try:
        await rover.offboard.start()
        print("🎮 Modo Offboard activado.")
    except OffboardError as e:
        print(f"❌ Error al activar Offboard: {e}")
        return

    print("📡 Escuchando posición del líder...")

    async def recibir_posiciones(sock):
        nonlocal leader_start_x, leader_start_y
        loop = asyncio.get_event_loop()
        while True:
            try:
                data, addr = await loop.run_in_executor(None, sock.recvfrom, 1024)
                mensaje = data.decode()
                print(f"he recibido: {mensaje}")

                x_str, y_str = mensaje.split(",")
                leader_x, leader_y = float(x_str), float(y_str)

                if leader_start_x is None:
                    leader_start_x = leader_x
                    leader_start_y = leader_y
                    print(f"📍 Posición inicial del líder: ({leader_x:.2f}, {leader_y:.2f})")
                    continue

                dx = leader_x - leader_start_x
                dy = leader_y - leader_start_y
                last_position["x"] = follower_start_x + dx
                last_position["y"] = follower_start_y + dy

                print(f"diferencia x: {dx:.2f}, diferencia y: {dy:.2f} . Objetivo seguidor: ({last_position['x']:.2f}, {last_position['y']:.2f})")

            except BlockingIOError:
                await asyncio.sleep(0.1)
                continue

    async def enviar_a_posicion():
        while True:
            await rover.offboard.set_position_ned(PositionNedYaw(
                last_position["x"],
                last_position["y"],
                -10.0,
                0.0
            ))
            print(f"enviando posición a ({last_position['x']:.2f}, {last_position['y']:.2f})")
            await asyncio.sleep(0.5)

    await asyncio.gather(
        recibir_posiciones(sock),
        enviar_a_posicion()
    )

if __name__ == "__main__":
    asyncio.run(follow_leader())
