import cv2
import numpy as np

def nada(x):
    # Función vacía para los trackbars, ya que getTrackbarPos() se encarga
    pass

ruta_imagen = 'imagen.png' # Asegúrate de que esta imagen exista en la misma carpeta
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

# --- Crear ventanas ---
cv2.namedWindow('Imagen Original', cv2.WINDOW_NORMAL) # Ventana para la imagen original
cv2.resizeWindow('Imagen Original', ancho_ventana, imagen_bgr.shape[0])
cv2.imshow('Imagen Original', imagen_bgr) # Mostrar la imagen original

cv2.namedWindow('Mascara', cv2.WINDOW_NORMAL) # Ventana para la máscara resultante
cv2.resizeWindow('Mascara', ancho_ventana, imagen_bgr.shape[0])

cv2.namedWindow('Controles HSV')
cv2.resizeWindow('Controles HSV', 400, 300) # Ajusta el tamaño de la ventana de controles

# --- Crear Trackbars ---
# Los valores iniciales son teoricamente para amarillo
cv2.createTrackbar('H Min', 'Controles HSV', 20, 179, nada)
cv2.createTrackbar('H Max', 'Controles HSV', 35, 179, nada)
cv2.createTrackbar('S Min', 'Controles HSV', 100, 255, nada)
cv2.createTrackbar('S Max', 'Controles HSV', 255, 255, nada)
cv2.createTrackbar('V Min', 'Controles HSV', 100, 255, nada)
cv2.createTrackbar('V Max', 'Controles HSV', 255, 255, nada)

print("Ajusta los trackbars hasta que la 'Mascara' muestre solo la cinta amarilla.")
print("Presiona 'q' para salir y mostrar los valores seleccionados.")

while True:
    # --- Obtener valores de los trackbars ---
    h_min = cv2.getTrackbarPos('H Min', 'Controles HSV')
    h_max = cv2.getTrackbarPos('H Max', 'Controles HSV')
    s_min = cv2.getTrackbarPos('S Min', 'Controles HSV')
    s_max = cv2.getTrackbarPos('S Max', 'Controles HSV')
    v_min = cv2.getTrackbarPos('V Min', 'Controles HSV')
    v_max = cv2.getTrackbarPos('V Max', 'Controles HSV')

    # Asegurarse de que H_min no sea mayor que H_max, etc. (buena práctica)
    # Aunque getTrackbarPos no lo permite directamente, esta comprobación protege
    # si los valores se cambian en un orden inusual o si se usaran variables externas.
    if h_min > h_max: h_min, h_max = h_max, h_min
    if s_min > s_max: s_min, s_max = s_max, s_min
    if v_min > v_max: v_min, v_max = v_max, v_min

    limite_inferior = np.array([h_min, s_min, v_min])
    limite_superior = np.array([h_max, s_max, v_max])

    # --- Aplicar máscara ---
    mascara = cv2.inRange(imagen_hsv, limite_inferior, limite_superior)

    # --- Mostrar la máscara ---
    cv2.imshow('Mascara', mascara)

    # --- Esperar una pulsación de tecla ---
    # cv2.waitKey(1) espera 1 milisegundo y devuelve el código ASCII de la tecla pulsada.
    # Si no se pulsa ninguna tecla, devuelve -1.
    # & 0xFF es para asegurar compatibilidad en diferentes sistemas operativos.
    key = cv2.waitKey(1) & 0xFF

    # --- Salir si se presiona 'q' ---
    if key == ord('q'): # ord('q') devuelve el valor ASCII de 'q'
        break

# --- Fuera del bucle: mostrar los valores finales ---
print("\nValores HSV seleccionados:")
print(f"H Min: {h_min}, H Max: {h_max}")
print(f"S Min: {s_min}, S Max: {s_max}")
print(f"V Min: {v_min}, V Max: {v_max}")

# --- Cerrar todas las ventanas de OpenCV ---
cv2.destroyAllWindows()