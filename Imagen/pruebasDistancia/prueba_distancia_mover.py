import asyncio
import numpy as np
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed, PositionNedYaw)

#Define las coordenadas NED del CENTRO DE LA APERTURA de la puerta objetivo
#Ejemplo para puerta_grande_1 del puertas_rojas.sdf
#La puerta esta colocada en: <pose>15 2 0 0 0 -0.5236</pose>
#Convertir a NED:
    #Norte = X_Gazebo = 15.0
    #Este = Y_Gazebo = 2.0
    #Abajo = -Z_Gazebo_Centro_Apertura = -1.55

GATE_POS_NED = np.array([15.0, 2.0, -1.55]) 

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Distancia Test (Hold): Esperando conexión...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Distancia Test (Hold): -- Conectado!")
            break
    
    print("Distancia Test (Hold): Esperando posición global...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Distancia Test (Hold): -- Posición global OK.")
            break
    
    print("Distancia Test (Hold): -- Armando dron")
    await drone.action.arm()
    await asyncio.sleep(1)

    print("Distancia Test (Hold): -- Despegando (Takeoff)...")
    await drone.action.takeoff()

    print("Distancia Test (Hold): -- Esperando que alcance altura (~2.5m)...")
    await asyncio.sleep(10) 
    print("Distancia Test (Hold): -- Iniciando medición de distancia...")

    pos_dron = np.array([0.0, 0.0, -2.5]) 

    while True:
        # --- Línea Corregida ---
        # Llama a la función correcta y espera el siguiente dato
        pos_vel_ned = await drone.telemetry.position_velocity_ned().__aiter__().__anext__()
        # Extrae solo la parte de la posición del objeto recibido
        position_ned = pos_vel_ned.position
        # --- Fin de la corrección ---

        pos_dron = np.array([position_ned.north_m, position_ned.east_m, position_ned.down_m])
        distancia = np.linalg.norm(pos_dron - GATE_POS_NED)
        print(f"Distancia REAL (Ground Truth): {distancia:.2f} m | Pos Dron NED: N={pos_dron[0]:.2f}, E={pos_dron[1]:.2f}, D={pos_dron[2]:.2f}")

        await asyncio.sleep(0.5) # Update rate

if __name__ == "__main__":
    print("Iniciando script medición distancia (Modo Takeoff/Hold)...")
    asyncio.run(run())
    print("Script terminado (Ctrl+C o error).")