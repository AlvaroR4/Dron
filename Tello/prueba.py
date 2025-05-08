import cv2
from djitellopy import Tello
import time

# --- Configuración ---
# No se necesita mucho para este script simple

print("Iniciando script simple de visualización del Tello...")

# 1. Crear objeto Tello
tello = Tello()

# 2. Conectar al Tello
print("Conectando al Tello...")
try:
    tello.connect()
    print("¡Conectado!")
    print(f"Nivel de batería: {tello.get_battery()}%")
except Exception as e:
    print(f"No se pudo conectar al Tello: {e}")
    exit() # Salir si no se puede conectar

# 3. Configurar stream de vídeo
print("Configurando stream de vídeo...")
try:
    # Asegurarse de que el stream esté apagado antes de iniciarlo
    tello.streamoff() 
    time.sleep(0.5) # Pequeña pausa
    tello.streamon()
    print("Stream de vídeo activado.")
    # Obtener el objeto que lee los frames en segundo plano
    frame_reader = tello.get_frame_read()
    # Esperar un poco para que el stream se estabilice
    time.sleep(1) 
except Exception as e:
    print(f"Error al configurar el stream: {e}")
    tello.end()
    exit()

print("Mostrando vídeo. Presiona 'q' en la ventana de vídeo para salir.")

# 4. Bucle principal para mostrar vídeo
try:
    while True:
        # Obtener el frame más reciente
        frame_actual = frame_reader.frame

        # Si no se recibe frame, esperar un poco y continuar
        if frame_actual is None:
            time.sleep(0.01)
            continue
        
        # Mostrar el frame en una ventana de OpenCV
        # Puedes descomentar la siguiente línea para redimensionar si es muy grande
        # frame_mostrable = cv2.resize(frame_actual, (640, 480))
        frame_mostrable = frame_actual # Mostrar tamaño original
        
        cv2.imshow("Video del Tello", frame_mostrable)

        # Esperar 1ms y comprobar si se pulsó la tecla 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Tecla 'q' presionada. Saliendo...")
            break

except Exception as e:
    print(f"Ocurrió un error durante la visualización: {e}")
except KeyboardInterrupt:
     print("Interrupción de teclado (Ctrl+C) detectada.")

# 5. Limpieza final (muy importante)
finally:
    print("Limpiando recursos...")
    # Destruir ventanas de OpenCV
    cv2.destroyAllWindows()
    
    # Detener el stream de vídeo
    print("Intentando detener stream...")
    try:
        tello.streamoff()
        print("Stream detenido.")
    except Exception as e_so:
        print(f"Error al detener stream: {e_so}")
        
    # Desconectar del Tello
    print("Intentando desconectar...")
    try:
        tello.end()
        print("Desconectado.")
    except Exception as e_end:
        print(f"Error al desconectar: {e_end}")
        
    print("Script finalizado.")