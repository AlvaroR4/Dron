"""
probarVideo2.py
Lee un archivo de video (video.mp4), aplica MiDaS para estimar profundidad,
muestra el mapa de profundidad coloreado, y escribe los valores normalizados (0-1)
de la profundidad en un archivo CSV.

Dependencias:
  pip install opencv-python torch torchvision torchaudio numpy
"""

import cv2
import torch
import numpy as np
import csv
import traceback # Para un mejor logging de errores

# --- Configuración ---
NOMBRE_ARCHIVO_VIDEO = "video.mp4" # Asegúrate que este archivo exista en la misma carpeta
# Modelo MiDaS a usar ("MiDaS_small", "MiDaS", "DPT_Large", "DPT_Hybrid")
TIPO_MODELO_MIDAS = "MiDaS_small"
# Mapa de color de OpenCV a usar (ej: cv2.COLORMAP_INFERNO, cv2.COLORMAP_JET, etc.)
MAPA_COLOR_SELECCIONADO = cv2.COLORMAP_INFERNO
# Nombre del archivo CSV para guardar los datos
NOMBRE_ARCHIVO_CSV = 'distancias_video_normalizadas.csv'
# Controlar la velocidad de reproducción (saltar N-1 frames). 1 para procesar todos.
PROCESAR_CADA_N_FRAMES = 1
# Escribir cabecera en CSV (dimensiones de la imagen)
ESCRIBIR_CABECERA_CSV = True


class VisualizadorMidasVideoCSV:
    def __init__(self):
        print(f"Iniciando Visualizador MiDaS con CSV para vídeo: {NOMBRE_ARCHIVO_VIDEO}")

        # Cargar modelo MiDaS y transformaciones una sola vez
        self.dispositivo = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"Cargando modelo MiDaS ({TIPO_MODELO_MIDAS}) en {self.dispositivo}...")
        try:
            self.midas = torch.hub.load("intel-isl/MiDaS", TIPO_MODELO_MIDAS)
            self.midas.to(self.dispositivo)
            self.midas.eval()

            transformaciones_midas = torch.hub.load("intel-isl/MiDaS", "transforms")
            if TIPO_MODELO_MIDAS == "MiDaS_small":
                self.transformacion = transformaciones_midas.small_transform
            elif "DPT" in TIPO_MODELO_MIDAS:
                self.transformacion = transformaciones_midas.dpt_transform
            else:
                self.transformacion = transformaciones_midas.dpt_transform
                print(f"Advertencia: Usando 'dpt_transform' para el modelo '{TIPO_MODELO_MIDAS}'. Verifica si es la transformación óptima.")

            print("Modelo MiDaS y transformaciones cargadas.")
        except Exception as e:
            print(f"Error fatal cargando modelo MiDaS: {e}. Asegúrate de tener conexión a internet la primera vez y la librería 'timm' instalada (`pip install timm`).")
            traceback.print_exc()
            raise SystemExit(f"Fallo al cargar MiDaS: {e}")

        # Crear ventana de OpenCV
        cv2.namedWindow("Mapa de Profundidad MiDaS (OpenCV)", cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow("Video Original", cv2.WINDOW_AUTOSIZE)


        # Inicializar el archivo CSV
        try:
            # Usamos 'w' para escribir desde cero cada vez, si quieres 'a' (append), ajusta la lógica de cabeceras
            self.archivo_csv = open(NOMBRE_ARCHIVO_CSV, 'w', newline='')
            self.escritor_csv = csv.writer(self.archivo_csv)
            self.cabecera_escrita = not ESCRIBIR_CABECERA_CSV # Si no se quiere cabecera, se marca como ya escrita
            self.contador_frames_csv = 0
            print(f"Archivo CSV '{NOMBRE_ARCHIVO_CSV}' abierto para escritura.")
        except IOError as e:
            print(f"Error fatal abriendo archivo CSV '{NOMBRE_ARCHIVO_CSV}': {e}")
            traceback.print_exc()
            # Si no podemos escribir el CSV, podríamos decidir si continuar solo con visualización o salir.
            # Por ahora, saldremos si el CSV es una parte crítica.
            raise SystemExit(f"Fallo al abrir CSV: {e}")


    def procesar_frame(self, frame_bgr_original, numero_frame_global):
        if frame_bgr_original is None:
            print("Frame original es None, no se puede procesar.")
            return

        try:
            frame_rgb = cv2.cvtColor(frame_bgr_original, cv2.COLOR_BGR2RGB)
        except cv2.error as e:
            print(f"Error CvBridge (cvtColor): {e}")
            return
        except Exception as e:
             print(f"Error convirtiendo imagen: {e}")
             return

        try:
            # Preprocesar imagen para MiDaS
            lote_entrada = self.transformacion(frame_rgb).to(self.dispositivo)

            # Realizar inferencia
            with torch.no_grad():
                prediccion = self.midas(lote_entrada)
                prediccion = torch.nn.functional.interpolate(
                    prediccion.unsqueeze(1),
                    size=frame_rgb.shape[:2],
                    mode="bicubic",
                    align_corners=False,
                ).squeeze()

            mapa_profundidad_raw = prediccion.cpu().numpy() # Estos son los valores "reales" de MiDaS (relativos)

            # Normalizar el mapa de profundidad raw al rango 0-1 para el CSV
            depth_min = np.min(mapa_profundidad_raw)
            depth_max = np.max(mapa_profundidad_raw)
            if depth_max > depth_min: # Evitar división por cero si el mapa es plano
                mapa_profundidad_normalizado_csv = (mapa_profundidad_raw - depth_min) / (depth_max - depth_min)
            else:
                mapa_profundidad_normalizado_csv = np.zeros_like(mapa_profundidad_raw)


            # Escribir los valores normalizados en el CSV
            if self.escritor_csv:
                if not self.cabecera_escrita:
                    altura, anchura = mapa_profundidad_normalizado_csv.shape
                    self.escritor_csv.writerow([f"Frame_{numero_frame_global}_Dim_{altura}x{anchura}"])
                    # No escribimos la cabecera de dimensiones para cada frame, solo una vez o al principio
                    # Para datos masivos, cada fila suele ser un frame, y la primera fila los nombres de columna
                    # Aquí, cada fila será un frame aplanado.
                    # Si quieres una cabecera con "pixel_1, pixel_2, ...", habría que generarla
                    self.cabecera_escrita = True # Marcamos que la cabecera (o la info del primer frame) ya se escribió

                # Separador entre datos de frames, o simplemente escribir la fila
                # self.escritor_csv.writerow([f"Frame_{numero_frame_global}_Data:"]) # Opcional
                self.escritor_csv.writerow(mapa_profundidad_normalizado_csv.flatten().tolist())
                self.archivo_csv.flush() # Asegurar que los datos se escriban (puede impactar rendimiento si es muy frecuente)
                self.contador_frames_csv += 1


            # --- Visualización con OpenCV ---
            # Usamos mapa_profundidad_raw para la visualización, normalizándolo a 0-255
            mapa_profundidad_normalizado_vis = cv2.normalize(mapa_profundidad_raw, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            mapa_calor_profundidad_bgr = cv2.applyColorMap(mapa_profundidad_normalizado_vis, MAPA_COLOR_SELECCIONADO)

            cv2.imshow("Video Original", frame_bgr_original)
            cv2.imshow("Mapa de Profundidad MiDaS (OpenCV)", mapa_calor_profundidad_bgr)
            return True

        except Exception as e:
            print(f"Error durante procesamiento MiDaS, CSV o visualización: {e}")
            traceback.print_exc()
            return False

    def cerrar(self):
        print("Cerrando visualizador MiDaS y archivo CSV...")
        cv2.destroyAllWindows()
        if hasattr(self, 'archivo_csv') and self.archivo_csv:
            self.archivo_csv.close()
            print(f"Archivo CSV cerrado. Se escribieron datos de {self.contador_frames_csv} frames.")

def main():
    visualizador_midas_csv = None
    captura_video = None
    contador_frames_total = 0
    contador_frames_procesados = 0

    try:
        visualizador_midas_csv = VisualizadorMidasVideoCSV()
        captura_video = cv2.VideoCapture(NOMBRE_ARCHIVO_VIDEO)

        if not captura_video.isOpened():
            print(f"Error: No se pudo abrir el archivo de video '{NOMBRE_ARCHIVO_VIDEO}'")
            return

        print("Procesando video. Presiona 'q' en la ventana de visualización para salir.")

        while captura_video.isOpened():
            ret, frame = captura_video.read()
            if not ret:
                print("Fin del video o error al leer frame.")
                break
            
            contador_frames_total +=1

            if contador_frames_total % PROCESAR_CADA_N_FRAMES != 0:
                cv2.imshow("Video Original", frame) # Mostrar aunque no se procese
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Salida solicitada por el usuario.")
                    break
                continue
            
            contador_frames_procesados +=1
            if not visualizador_midas_csv.procesar_frame(frame, contador_frames_procesados):
                 print("Error en el procesamiento del frame, saliendo...")
                 break

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Salida solicitada por el usuario.")
                break
    except SystemExit as se:
        print(f"Salida del sistema: {se}")
    except KeyboardInterrupt:
        print("\nCtrl+C detectado, cerrando programa...")
    except Exception as e:
        print(f"Error inesperado en main: {e}")
        traceback.print_exc()
    finally:
        if captura_video:
            captura_video.release()
            print("Recurso de video liberado.")
        if visualizador_midas_csv:
            visualizador_midas_csv.cerrar()
        print(f"Programa finalizado. Total frames leídos: {contador_frames_total}, procesados: {contador_frames_procesados}.")


if __name__ == '__main__':
    main()