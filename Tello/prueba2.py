# test_display_opencv.py
import cv2
import numpy as np

print("DEBUG: Creando imagen de prueba...")
img = np.zeros((300, 400, 3), dtype=np.uint8)
cv2.putText(img, 'Prueba OpenCV', (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

print("DEBUG: Llamando a cv2.imshow()...")
cv2.imshow("Prueba Minima OpenCV", img)
print("DEBUG: Llamando a cv2.waitKey(0)... (Presiona una tecla en la ventana si aparece)")
key = cv2.waitKey(0)
print(f"DEBUG: Tecla presionada: {key}")
cv2.destroyAllWindows()
print("DEBUG: Ventanas destruidas. Fin.")