#!/usr/bin/env python3
import asyncio
from mavsdk import System

async def ejecutar(rover: System):
    print("Esperando conexión al rover...")
    await rover.connect(system_address="udp://:14540")
    
    # Verificación opcional de conexión
    async for estado in rover.core.connection_state():
        if estado.is_connected:
            print("Rover conectado")
            break

    print("Armando el rover...")
    await rover.action.arm()
    await asyncio.sleep(1)

    # Parámetros de la trayectoria
    velocidad_avance = 0.5  # Velocidad hacia adelante (m/s)
    valor_giro = 0.5        # Valor para el giro (ajustar según la simulación)
    tiempo_avance = 2.0     # Tiempo para avanzar 1 m (según la velocidad)
    tiempo_giro = 1.0       # Tiempo para girar 90° (ajustar experimentalmente)

    # --- Trayectoria en cuadrado ---
    for i in range(4):
        print(f"Segmento {i+1}: Avanzar 1 m")
        await rover.manual_control.set_manual_control_input(velocidad_avance, 0.0, 0.0, 0.0)
        await asyncio.sleep(tiempo_avance)
        await rover.manual_control.set_manual_control_input(0.0, 0.0, 0.0, 0.0)
        await asyncio.sleep(1)

        print(f"Segmento {i+1}: Girar 90° a la derecha")
        await rover.manual_control.set_manual_control_input(0.0, valor_giro, 0.0, 0.0)
        await asyncio.sleep(tiempo_giro)
        await rover.manual_control.set_manual_control_input(0.0, 0.0, 0.0, 0.0)
        await asyncio.sleep(1)

    print("Deteniendo el rover...")
    await rover.action.disarm()
    print("Trayectoria completada.")

if __name__ == "__main__":
    # Se asume que el servidor MAVSDK está corriendo en localhost:50050
    rover = System(mavsdk_server_address="localhost", port=50050)
    asyncio.get_event_loop().run_until_complete(ejecutar(rover))
