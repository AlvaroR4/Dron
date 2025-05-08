"""
probarVideo.py
Lee un archivo de video (video.mp4), aplica MiDaS para estimar profundidad
y muestra el mapa de profundidad coloreado en tiempo real usando OpenCV.

Dependencias:
  pip install opencv-python torch torchvision torchaudio numpy
  (Asegúrate de tener un entorno donde PyTorch pueda acceder a CUDA si tienes GPU)
"""

import cv2
import torch
import numpy as np
import traceback # Para un mejor logging de errores

# --- Configuración ---
NOMBRE_ARCHIVO_VIDEO = "video.mp4" # Asegúrate que este archivo exista en la misma carpeta
# Modelo MiDaS a usar ("MiDaS_small", "MiDaS", "DPT_Large", "DPT_Hybrid")
TIPO_MODELO_MIDAS = "MiDaS_small"
# Mapa de color de OpenCV a usar (ej: cv2.COLORMAP_INFERNO, cv2.COLORMAP_JET, etc.)
MAPA_COLOR_SELECCIONADO = cv2.COLORMAP_INFERNO
# Controlar la velocidad de reproducción (saltar N-1 frames). 1 para procesar todos.
PROCESAR_CADA_N_FRAMES = 1


class VisualizadorMidasVideo:
    def __init__(self):
        print(f"Iniciando Visualizador MiDaS para vídeo: {NOMBRE_ARCHIVO_VIDEO}")

        # Cargar modelo MiDaS y transformaciones una sola vez
        self.dispositivo = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"Cargando modelo MiDaS ({TIPO_MODELO_MIDAS}) en {self.dispositivo}...")
        try:
            # Intel ISL MiDaS es el repositorio correcto
            self.midas = torch.hub.load("intel-isl/MiDaS", TIPO_MODELO_MIDAS)
            self.midas.to(self.dispositivo)
            self.midas.eval()

            # Cargar las transformaciones adecuadas para el modelo
            # El repositorio de transformaciones también es "intel-isl/MiDaS"
            transformaciones_midas = torch.hub.load("intel-isl/MiDaS", "transforms")
            if TIPO_MODELO_MIDAS == "MiDaS_small":
                self.transformacion = transformaciones_midas.small_transform
            elif "DPT" in TIPO_MODELO_MIDAS: # DPT_Large, DPT_Hybrid
                self.transformacion = transformaciones_midas.dpt_transform
            else: # Modelo MiDaS base
                # Para el modelo MiDaS original (no DPT y no small),
                # la documentación sugiere usar DPT transform si no hay una específica,
                # o referirse a cómo se usaba antes (puede requerir una transformación más genérica).
                # Por seguridad y compatibilidad con los ejemplos de MiDaS:
                self.transformacion = transformaciones_midas.dpt_transform
                # Si usas el modelo "MiDaS" (el original, más grande que small),
                # verifica si esta transformación es la óptima o si requiere una diferente.
                print(f"Advertencia: Usando 'dpt_transform' para el modelo '{TIPO_MODELO_MIDAS}'. Verifica si es la transformación óptima.")


            print("Modelo MiDaS y transformaciones cargadas.")
        except Exception as e:
            print(f"Error fatal cargando modelo MiDaS: {e}. Asegúrate de tener conexión a internet la primera vez y la librería 'timm' instalada (`pip install timm`).")
            traceback.print_exc()
            # No podemos continuar si el modelo no carga
            raise SystemExit(f"Fallo al cargar MiDaS: {e}")


        # Crear ventana de OpenCV
        cv2.namedWindow("Mapa de Profundidad MiDaS (OpenCV)", cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow("Video Original", cv2.WINDOW_AUTOSIZE)


    def procesar_frame(self, frame_bgr_original):
        if frame_bgr_original is None:
            print("Frame original es None, no se puede procesar.")
            return

        try:
            # Convertir a RGB para MiDaS
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

            # Mover a CPU y convertir a NumPy
            mapa_profundidad = prediccion.cpu().numpy()

            # --- Visualización con OpenCV ---
            # 1. Normalizar el mapa de profundidad al rango 0-255 (para 8 bits)
            mapa_profundidad_normalizado_vis = cv2.normalize(mapa_profundidad, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

            # 2. Aplicar un mapa de color para visualizar
            mapa_calor_profundidad_bgr = cv2.applyColorMap(mapa_profundidad_normalizado_vis, MAPA_COLOR_SELECCIONADO)

            # 3. Mostrar las imagenes resultantes
            cv2.imshow("Video Original", frame_bgr_original)
            cv2.imshow("Mapa de Profundidad MiDaS (OpenCV)", mapa_calor_profundidad_bgr)
            
            # Retornar True si el procesamiento fue exitoso para la tecla de salida
            return True

        except Exception as e:
            print(f"Error durante procesamiento MiDaS o visualización OpenCV: {e}")
            traceback.print_exc()
            return False # Indica que hubo un error y se podría querer salir

    def cerrar(self):
        print("Cerrando visualizador MiDaS...")
        cv2.destroyAllWindows() # Cerrar ventanas de OpenCV

def main():
    visualizador_midas = None
    captura_video = None
    contador_frames = 0
    try:
        visualizador_midas = VisualizadorMidasVideo()
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
            
            contador_frames += 1
            if contador_frames % PROCESAR_CADA_N_FRAMES != 0:
                # Mostrar el frame original aunque no se procese para mantener la tasa de FPS visual
                cv2.imshow("Video Original", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Salida solicitada por el usuario.")
                    break
                continue


            if not visualizador_midas.procesar_frame(frame):
                # Si procesar_frame devuelve False (por un error), podríamos querer salir
                print("Error en el procesamiento del frame, saliendo...")
                break
            
            # cv2.waitKey(1) es crucial para que OpenCV procese eventos de ventana y refresque
            # Si se presiona 'q', se sale del bucle
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
        if visualizador_midas:
            visualizador_midas.cerrar()
        print("Programa finalizado.")

if __name__ == '__main__':
    main()