"""
DEPENDENCIAS:
pip install djitellopy opencv-python numpy
sudo apt update
sudo apt install ffmpeg

pip install pynput

pip install djitellopy opencv-python numpy pynput
"""


import cv2
import numpy as np
import socket
import time
from djitellopy import Tello

SERVER_IP = "127.0.0.1" 
SERVER_PORT = 65432
TAMANO_REAL_PUERTA_M = 1.5
DISTANCIA_FOCAL_PIXELS = 920 
MIN_CONTOUR_AREA = 50
AREA_UMBRAL_GRANDE = 100

# Explicación de los 4 Límites de Color (HSV para Rojo):
# 1. Se usa el espacio HSV (Tono, Saturación, Valor) por ser más robusto a cambios de luz que BGR.
# 2. El Tono (H) del Rojo "da la vuelta" en la escala 0-179 de OpenCV (está al principio y al final).
# 3. Por eso, se definen DOS rangos (uno bajo 0-10 y otro alto 160-179) y se combinan con OR para capturar todos los rojos.

COLOR_LOWER_1 = np.array([0, 100, 100])
COLOR_UPPER_1 = np.array([10, 255, 255])
COLOR_LOWER_2 = np.array([160, 100, 100])
COLOR_UPPER_2 = np.array([180, 255, 255])


def calcular_distancia(tamanio_ancho_pixels):
    if tamanio_ancho_pixels <= 1 or DISTANCIA_FOCAL_PIXELS is None:
        return float('inf')
    return (TAMANO_REAL_PUERTA_M * DISTANCIA_FOCAL_PIXELS) / tamanio_ancho_pixels

def main():
    tello = Tello()
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_address = (SERVER_IP, SERVER_PORT)
    print(f"Procesador Tello: Conectando al dron...")

    try:
        tello.connect()
        print(f"Batería: {tello.get_battery()}%")
        tello.streamon()
        frame_read = tello.get_frame_read()
        print("Stream de video iniciado. Enviando datos a UDP {SERVER_IP}:{SERVER_PORT}")

        while True:
            if frame_read.stopped:
                print("Stream detenido.")
                break

            current_frame = frame_read.frame
            if current_frame is None:
                time.sleep(0.01)
                continue

            img_display = current_frame.copy()
            height, width, _ = img_display.shape
            center_x = width // 2
            center_y = height // 2

            hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

            # Crear máscara para el rojo
            mask1 = cv2.inRange(hsv, COLOR_LOWER_1, COLOR_UPPER_1)
            mask2 = cv2.inRange(hsv, COLOR_LOWER_2, COLOR_UPPER_2)
            mask = cv2.bitwise_or(mask1, mask2)


            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            detected_targets = []
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > MIN_CONTOUR_AREA:
                    M = cv2.moments(contour)
                    if M['m00'] > 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])

                        if area > AREA_UMBRAL_GRANDE:
                            color_centroide = current_frame[cY, cX]
                            if mask[cY, cX] > 0: 
                                 print(f"Advertencia: Detección grande (área={area:.0f}) con centroide ({cX},{cY}) en zona roja. ¿Poste?")

                        rect = cv2.minAreaRect(contour)
                        (w_rot, h_rot) = rect[1]
                        tamanio_ancho_pixels = float(min(w_rot, h_rot)) 

                        distancia_estimada = calcular_distancia(tamanio_ancho_pixels)
                        offset_x = cX - center_x
                        offset_y = cY - center_y

                        detected_targets.append({
                            'offset_x': offset_x, 'offset_y': offset_y,
                            'distance': distancia_estimada, 'centroide': (cX, cY),
                            'rect': rect
                        })

            target_offset_x, target_offset_y, target_distance = 0, 0, 0.0
            num_targets_detected = len(detected_targets)
            closest_target = None

            if num_targets_detected > 0:
                detected_targets.sort(key=lambda t: t['distance'])
                closest_target = detected_targets[0]
                target_offset_x = closest_target['offset_x']
                target_offset_y = closest_target['offset_y']
                target_distance = closest_target['distance']

            message = f"{target_offset_x},{target_offset_y},{target_distance:.2f},{num_targets_detected}"
            try:
                udp_socket.sendto(message.encode('utf-8'), server_address)
            except Exception as e:
                print(f"Error enviando UDP: {e}")

            for i, target in enumerate(detected_targets):
                color = (0, 255, 0)
                if target == closest_target: color = (255, 0, 0)
                box_points = cv2.boxPoints(target['rect'])
                cv2.drawContours(img_display, [np.intp(box_points)], 0, color, 2)
                cv2.circle(img_display, target['centroide'], 5, color, -1)
                info_text = f"D:{target['distance']:.1f}"
                cv2.putText(img_display, info_text, (target['centroide'][0] + 10, target['centroide'][1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

            cv2.imshow("Deteccion Tello", img_display)
            if cv2.waitKey(1) & 0xFF == ord('q'): # Salir con 'q'
                 break

    except KeyboardInterrupt:
        print("--> Ctrl+C detectado.")
    except Exception as e:
        print(f"Error principal: {e}")
    finally:
        print("--> Limpiando...")
        cv2.destroyAllWindows()
        tello.streamoff()
        udp_socket.close()
        print('Socket UDP cerrado.')
        print("--> Programa de procesamiento finalizado.")

if __name__ == '__main__':
    main()