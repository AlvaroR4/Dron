import cv2
import numpy as np
import socket
import time
import math # Para calcular distancia si usas funciones trigonométricas o similar
import threading # Importado por si alguna parte de Tello lo usa implícitamente
from djitellopy import Tello

# --- Constantes de Comunicación UDP (para enviar a moverDron_tello.py) ---
SERVER_IP = "127.0.0.1"
SERVER_PORT = 65432

# --- Constantes de Visión (¡DEBES AJUSTAR Y CALIBRAR ESTAS!) ---
TAMANO_REAL_PUERTA_M = 1.5  # Ancho real de tu puerta en metros
DISTANCIA_FOCAL_PIXELS_TELLO = 920 # ¡EJEMPLO! CALIBRA ESTO PARA TU TELLO Y RESOLUCIÓN
MIN_CONTOUR_AREA_TELLO = 3000 # Área mínima en píxeles para considerar un contorno. AJUSTA ESTO.

# Rangos HSV para ROJO (¡DEBES AJUSTARLOS CON UN CALIBRADOR!)
# Rango 1 (tonos de rojo más bajos en Hue)
COLOR_LOWER_1 = np.array([0, 120, 90])
COLOR_UPPER_1 = np.array([10, 255, 255])
# Rango 2 (tonos de rojo más altos en Hue)
COLOR_LOWER_2 = np.array([165, 120, 90])
COLOR_UPPER_2 = np.array([179, 255, 255])

# --- Constantes para el bucle de visualización y procesamiento ---
FRAME_WIDTH_PROC = 640  # Ancho al que se redimensionará la imagen para procesar
FRAME_HEIGHT_PROC = 480 # Alto al que se redimensionará la imagen para procesar
WAITKEY_DELAY_MS = 20   # Milisegundos para cv2.waitKey().
FRAMES_TO_SKIP_DISPLAY = 5 # Frames iniciales que no se mostrarán (para estabilización)
FRAMES_TIMEOUT_NONE = 600 # Iteraciones con frame=None antes de salir

# --- Parámetros del procesamiento de imagen (del snippet, puedes ajustarlos) ---
THRESHOLD1_CANNY = 166 # Umbral inferior para Canny
THRESHOLD2_CANNY = 171 # Umbral superior para Canny


def calcular_distancia(tamanio_aparente_pixels, distancia_focal_pixels, tamano_real_objeto_m):
    """Calcula la distancia a un objeto basado en el principio de triángulos semejantes."""
    if tamanio_aparente_pixels <= 1: # Evitar división por cero o valores ínfimos
        return float('inf')
    # Fórmula: Distancia = (TamañoRealObjeto_m * DistanciaFocal_px) / TamañoAparenteObjeto_px
    distancia_estimada_m = (tamano_real_objeto_m * distancia_focal_pixels) / tamanio_aparente_pixels
    return distancia_estimada_m


def main():
    print("DEBUG: Iniciando script 'tratarImagen_tello.py'...")
    
    tello = Tello()
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_address = (SERVER_IP, SERVER_PORT)

    frame_counter_validos = 0
    loop_iterations = 0
    none_frame_streak = 0

    try:
        print("DEBUG: Conectando al Tello...")
        tello.connect()
        print(f"DEBUG: Conectado. Batería: {tello.get_battery()}%")

        print("DEBUG: Iniciando stream de vídeo (tello.streamon())...")
        tello.streamon()
        print("DEBUG: Comando streamon enviado.")

        print("DEBUG: Obteniendo frame_reader (tello.get_frame_read())...")
        frame_reader = tello.get_frame_read()
        if frame_reader is None:
            print("ERROR CRÍTICO: frame_reader es None. Saliendo.")
            if 'tello' in locals(): tello.end()
            return
        print("DEBUG: frame_reader obtenido.")

        print(f"DEBUG: Pausa de 1.5 segundos para estabilizar el stream de vídeo...")
        time.sleep(1.5)

        print("DEBUG: Intentando un 'priming read' del frame_reader...")
        try:
            _ = frame_reader.frame 
            print("DEBUG: 'Priming read' accedido.")
            time.sleep(0.1)
        except Exception as e_prime:
            print(f"DEBUG: Excepción durante el 'priming read': {e_prime}")

        print("DEBUG: Entrando al bucle principal de lectura y procesamiento de frames...")
        while True:
            loop_iterations += 1

            if frame_reader.stopped:
                print(f"ERROR: Lector de frames detenido en iteración {loop_iterations}.")
                break

            current_frame = frame_reader.frame

            if current_frame is None:
                none_frame_streak += 1
                if none_frame_streak == 1 or none_frame_streak % 30 == 0:
                    print(f"DEBUG: Iter {loop_iterations}: current_frame es None. (Streak: {none_frame_streak})")
                    print(f"DEBUG: Lector frame_read.stopped: {frame_reader.stopped}")
                    if hasattr(frame_reader, 'thread') and frame_reader.thread is not None:
                        print(f"DEBUG: Hilo frame_reader vivo: {frame_reader.thread.is_alive()}")
                
                if none_frame_streak > FRAMES_TIMEOUT_NONE:
                     print(f"ERROR: {FRAMES_TIMEOUT_NONE} frames None. Saliendo.")
                     break
                time.sleep(0.02)
                continue
            
            none_frame_streak = 0
            frame_counter_validos += 1

            # Redimensionar el frame para un procesamiento consistente
            img_resized = cv2.resize(current_frame, (FRAME_WIDTH_PROC, FRAME_HEIGHT_PROC))
            img_contour_display = img_resized.copy() # Para dibujar encima

            # 1. Segmentación por Color (Rojo)
            img_hsv = cv2.cvtColor(img_resized, cv2.COLOR_BGR2HSV)
            mask_rojo1 = cv2.inRange(img_hsv, COLOR_LOWER_1, COLOR_UPPER_1)
            mask_rojo2 = cv2.inRange(img_hsv, COLOR_LOWER_2, COLOR_UPPER_2)
            final_mask = cv2.bitwise_or(mask_rojo1, mask_rojo2)

            # Aplicar la máscara a la imagen original (como en Snippet 3)
            result_masked_color = cv2.bitwise_and(img_resized, img_resized, mask=final_mask)

            # 2. Procesamiento adicional para encontrar contornos (del Snippet 3)
            img_blur = cv2.GaussianBlur(result_masked_color, (7, 7), 1)
            img_gray = cv2.cvtColor(img_blur, cv2.COLOR_BGR2GRAY)
            img_canny = cv2.Canny(img_gray, THRESHOLD1_CANNY, THRESHOLD2_CANNY)
            kernel_dilate = np.ones((5, 5)) # Kernel para dilatación
            img_dilated = cv2.dilate(img_canny, kernel_dilate, iterations=1)
            
            # 3. Encontrar Contornos
            contours, _ = cv2.findContours(img_dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            detected_targets_info = []
            frame_center_x = FRAME_WIDTH_PROC // 2
            frame_center_y = FRAME_HEIGHT_PROC // 2

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > MIN_CONTOUR_AREA_TELLO:
                    M = cv2.moments(cnt)
                    if M['m00'] > 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        offset_x = cx - frame_center_x
                        # En OpenCV, Y aumenta hacia abajo. Si el centroide (cy) es > frame_center_y,
                        # el objeto está DEBAJO del centro. Para que tu `moverDron` suba (ud positivo),
                        # este offset_y positivo está bien.
                        offset_y = cy - frame_center_y

                        # Usar minAreaRect para obtener dimensiones más robustas a la rotación
                        rect = cv2.minAreaRect(cnt) 
                        # rect es ((centro_x, centro_y), (ancho, alto), angulo_rotacion)
                        (box_w, box_h) = rect[1]
                        # Asumimos que nos interesa la dimensión más pequeña como "ancho aparente"
                        # o la que corresponda al TAMANO_REAL_PUERTA_M (si es ancho, usa ancho, si es alto, usa alto)
                        # Si tu TAMANO_REAL_PUERTA_M es el ancho, y la puerta puede estar rotada:
                        # Para una puerta que es más ancha que alta, y puede estar un poco rotada,
                        # min(box_w, box_h) puede no ser siempre el ancho percibido.
                        # Considera si la puerta siempre estará mayormente vertical.
                        # Si la puerta está mayormente vertical, box_w sería el ancho aparente.
                        # Si puede rotar mucho, min(box_w, box_h) es una aproximación.
                        # Vamos a usar box_w asumiendo que es el ancho y la puerta no está muy tumbada:
                        ancho_aparente_pixels = box_w # o min(box_w, box_h) si prefieres
                        if ancho_aparente_pixels < 1: ancho_aparente_pixels = 1 # Evitar problemas

                        distancia = calcular_distancia(ancho_aparente_pixels,
                                                       DISTANCIA_FOCAL_PIXELS_TELLO,
                                                       TAMANO_REAL_PUERTA_M)

                        detected_targets_info.append({
                            'cx': cx, 'cy': cy, 'offset_x': offset_x, 'offset_y': offset_y,
                            'distance': distancia, 'area': area, 'rect_obj': rect
                        })
            
            # 4. Seleccionar Objetivo y Preparar Mensaje UDP
            udp_offset_x, udp_offset_y, udp_distancia = 0.0, 0.0, float('inf') # Infinito si no hay target
            num_valid_targets = len(detected_targets_info)
            closest_target_info = None

            if num_valid_targets > 0:
                detected_targets_info.sort(key=lambda t: t['distance'])
                closest_target_info = detected_targets_info[0]
                udp_offset_x = float(closest_target_info['offset_x'])
                udp_offset_y = float(closest_target_info['offset_y'])
                udp_distancia = closest_target_info['distance']

                # Dibujar el objetivo más cercano en img_contour_display
                points = cv2.boxPoints(closest_target_info['rect_obj'])
                points = np.intp(points)
                cv2.drawContours(img_contour_display, [points], -1, (0, 255, 0), 2)
                cv2.circle(img_contour_display, (closest_target_info['cx'], closest_target_info['cy']), 5, (0, 255, 0), -1)
                cv2.putText(img_contour_display, f"D:{closest_target_info['distance']:.1f}m",
                            (closest_target_info['cx'] - 30, closest_target_info['cy'] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            if frame_counter_validos % 60 == 0 or num_valid_targets > 0 : # Imprimir si hay target o cada 60 frames
                print(f"DEBUG: Frame {frame_counter_validos}, Targets: {num_valid_targets}, Closest D: {udp_distancia:.2f}, Offset:({udp_offset_x},{udp_offset_y})")

            message = f"{udp_offset_x},{udp_offset_y},{udp_distancia:.2f},{num_valid_targets}"
            try:
                udp_socket.sendto(message.encode('utf-8'), server_address)
            except Exception as e_udp:
                if frame_counter_validos % 60 == 0:
                    print(f"WARN: Error enviando UDP: {e_udp}")

            # 5. Visualización
            display_image_final = img_contour_display
            if frame_counter_validos > FRAMES_TO_SKIP_DISPLAY:
                cv2.imshow("Tello Vision Processing", display_image_final)
            
            key_pressed = cv2.waitKey(WAITKEY_DELAY_MS) & 0xFF
            if key_pressed == ord('q'):
                print("DEBUG: Tecla 'q' presionada. Saliendo del bucle.")
                break
            
    except KeyboardInterrupt:
        print("--> Ctrl+C detectado. Terminando...")
    except Exception as e:
        print(f"Error principal en el script: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("--> Bloque finally: Limpiando recursos...")
        cv2.destroyAllWindows()
        print("DEBUG: Ventanas de OpenCV destruidas.")
        
        if 'tello' in locals():
            if hasattr(tello, 'is_flying') and tello.is_flying: 
                 print("WARN: El dron podría estar volando. Este script NO lo aterrizará.")
            
            print("DEBUG: Intentando tello.streamoff()...")
            try:
                tello.streamoff()
                print("DEBUG: streamoff completado.")
            except Exception as e_so:
                print(f"WARN: Excepción durante tello.streamoff(): {e_so}")
            
            print("DEBUG: Intentando tello.end()...")
            try:
                tello.end()
                print("DEBUG: end completado.")
            except Exception as e_end:
                print(f"WARN: Excepción durante tello.end(): {e_end}")

        if 'udp_socket' in locals() and udp_socket:
            udp_socket.close()
            print('DEBUG: Socket UDP cerrado.')
        
        print("--> Limpieza finalizada. Programa terminado.")

if __name__ == '__main__':
    main()