import cv2
import numpy as np

def nada(x):
    pass

ruta_imagen = 'imagen.jpg'
ancho_ventana = 640 

imagen_bgr_original = cv2.imread(ruta_imagen)

if imagen_bgr_original is None:
    print(f"Error: No se pudo cargar la imagen desde '{ruta_imagen}'.")
    print("Asegúrate de que la imagen exista y la ruta sea correcta.")
    exit()

altura_original, ancho_original = imagen_bgr_original.shape[:2]
if ancho_original > ancho_ventana:
    ratio = ancho_ventana / float(ancho_original)
    nueva_altura = int(altura_original * ratio)
    imagen_bgr = cv2.resize(imagen_bgr_original, (ancho_ventana, nueva_altura))
else:
    imagen_bgr = imagen_bgr_original.copy()

imagen_hsv = cv2.cvtColor(imagen_bgr, cv2.COLOR_BGR2HSV)
cv2.namedWindow('Controles HSV')
cv2.resizeWindow('Controles HSV', 400, 300) # Ajusta el tamaño de la ventana de controles

# Los valores iniciales son teoricamente para amarillo
# H (Hue/Tono): 0-179 en OpenCV
# S (Saturation/Saturación): 0-255
# V (Value/Brillo): 0-255
cv2.createTrackbar('H Min', 'Controles HSV', 20, 179, nada)
cv2.createTrackbar('H Max', 'Controles HSV', 35, 179, nada) 
cv2.createTrackbar('S Min', 'Controles HSV', 100, 255, nada) # Saturación mínima para evitar grises/blancos
cv2.createTrackbar('S Max', 'Controles HSV', 255, 255, nada) # Saturación máxima siempre 255
cv2.createTrackbar('V Min', 'Controles HSV', 100, 255, nada) # Brillo mínimo para evitar negros/oscuros
cv2.createTrackbar('V Max', 'Controles HSV', 255, 255, nada) # Brillo máximo siempre 255

print("Ajusta los trackbars hasta que la 'Mascara' muestre solo la cinta amarilla.")
print("Presiona 'q' para salir y mostrar los valores seleccionados.")

while True:
    h_min = cv2.getTrackbarPos('H Min', 'Controles HSV')
    h_max = cv2.getTrackbarPos('H Max', 'Controles HSV')
    s_min = cv2.getTrackbarPos('S Min', 'Controles HSV')
    s_max = cv2.getTrackbarPos('S Max', 'Controles HSV')
    v_min = cv2.getTrackbarPos('V Min', 'Controles HSV')
    v_max = cv2.getTrackbarPos('V Max', 'Controles HSV')
    limite_inferior_amarillo = np.array([h_min, s_min, v_min])
    limite_superior_amarillo = np.array([h_max, s_max, v_max])
    mascara = cv2.inRange(imagen_hsv, limite_inferior_amarillo, limite_superior_amarillo)

