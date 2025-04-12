import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import socket
import threading
import time # Añadido para el wait

# Ya no necesitas MAVSDK aquí si solo procesas imagen y envías UDP
# from mavsdk import System
# drone = System()

SERVER_IP = "127.0.0.1"
SERVER_PORT = 65432


class ProcesadorImagen(Node): # Renombrado para mayor claridad
    def __init__(self):
        super().__init__('procesador_imagen_real') # Nombre del nodo actualizado

        # --- Parámetro para el nombre del topic de entrada ---
        self.declare_parameter('topic_entrada_imagen', '/camara_real/image_raw')
        topic_entrada = self.get_parameter('topic_entrada_imagen').get_parameter_value().string_value
        self.get_logger().info(f"Suscribiéndose al topic de imagen: '{topic_entrada}'")

        # --- Modificación Principal: Cambiar el topic de suscripción ---
        self.subscription = self.create_subscription(
            Image,
            # '/world/default/model/x500_mono_cam_0/link/camera_link/sensor/imager/image', # <-- Topic ANTIGUO (del puente o Gazebo)
            topic_entrada, # <-- Topic NUEVO (publicado por publicador_camara.py)
            self.listener_callback,
            10) # QoS profile, 10 es un valor común para "keep last"
        self.br = CvBridge()

        # --- Variables para compartir la última posición ---
        self.latest_offset_x = 0
        self.latest_offset_y = 0
        self.offset_lock = threading.Lock() # Lock para acceso seguro entre hilos
        self._stop_event = threading.Event() # Evento para detener el hilo limpiamente

        # --- Iniciar el hilo para enviar posición en segundo plano ---
        self.sender_thread = threading.Thread(target=self.background_sender, daemon=True)
        self.sender_thread.start()
        self.get_logger().info('Hilo de envío UDP en segundo plano iniciado.')

    def listener_callback(self, data):
        # El logger ahora indica que procesa, no solo recibe
        self.get_logger().debug('Procesando frame de vídeo recibido...')
        try:
            # --- Conversión de Imagen (sin cambios) ---
            # Nota: cv_bridge generalmente devuelve BGR por defecto desde ROS Image
            current_frame_bgr = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except Exception as e:
             self.get_logger().error(f"Fallo en conversión de CvBridge: {e}")
             return

        # --- Procesamiento de Imagen (sin cambios en la lógica central) ---
        # imgRGB = cv2.cvtColor(current_frame_bgr, cv2.COLOR_BGR2RGB) # Convertir a RGB si es necesario para mostrar o procesar
        # Usaremos BGR directamente ya que tu máscara original usaba current_frame (que era BGR)
        img_display = current_frame_bgr.copy() # Copia para dibujar encima sin afectar el original si se usara más
        height, width, _ = current_frame_bgr.shape
        center_x = width // 2
        center_y = height // 2

        # Define los límites para el color ROJO en BGR
        # OJO: Estos valores [0,0,0] a [255,0,0] buscan AZUL PURO en BGR.
        # Para ROJO PURO en BGR sería: lower=[0, 0, 200], upper=[50, 50, 255] (aprox)
        # Ajusta estos valores según el color REAL que quieras detectar
        lower_color = np.array([0, 0, 150])  # Ejemplo para rojo (ajusta según necesites)
        upper_color = np.array([80, 80, 255]) # Ejemplo para rojo (ajusta según necesites)

        mask = cv2.inRange(current_frame_bgr, lower_color, upper_color)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        found_target = False
        current_offset_x = 0 # Resetea offset si no se encuentra nada
        current_offset_y = 0

        # --- Búsqueda de Contornos (sin cambios en la lógica) ---
        if contours: # Solo procesa si hay algún contorno
            # Opcional: encontrar el contorno más grande
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            # Evitar división por cero y filtrar por área mínima
            if M['m00'] > 100:
                found_target = True
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # Dibuja en la copia de la imagen
                cv2.circle(img_display, (cX, cY), 7, (0, 255, 0), -1) # Verde para el centro detectado
                cv2.drawContours(img_display, [c], -1, (0, 255, 0), 3) # Verde para el contorno

                current_offset_x = cX - center_x
                current_offset_y = cY - center_y

                cv2.putText(img_display, f"x_off: {current_offset_x}, y_off: {current_offset_y}", (cX - 50, cY - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2) # Verde

        # --- Actualizar Variables Compartidas (con lock) ---
        with self.offset_lock:
            self.latest_offset_x = current_offset_x
            self.latest_offset_y = current_offset_y
            if found_target:
                self.get_logger().debug(f'Objetivo encontrado. Offset actualizado: {current_offset_x}, {current_offset_y}')
            # else: # Opcional: loguear si no se encontró
                 # self.get_logger().debug('Objetivo no encontrado en este frame.')

        # --- Mostrar Imagen Procesada (sin cambios) ---
        # Dibuja el centro de la imagen como referencia
        cv2.circle(img_display, (center_x, center_y), 5, (0, 0, 255), -1) # Rojo para el centro real
        cv2.imshow("Imagen Procesada (Real)", img_display)
        # cv2.imshow("Mascara", mask) # Descomenta si quieres ver la máscara
        cv2.waitKey(1) # Muy importante para que imshow funcione

    def background_sender(self):
        """Función ejecutada por el hilo para enviar la posición UDP periódicamente."""
        # Esperar un poco antes de empezar a enviar para dar tiempo a que todo arranque
        time.sleep(2.0)
        self.get_logger().info(f'Empezando envío UDP a {SERVER_IP}:{SERVER_PORT}')
        # Usar un socket que se cierra automáticamente al salir del with
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            self.get_logger().info('Socket UDP del hilo de envío creado.')
            while not self._stop_event.is_set():
                with self.offset_lock:
                    current_x = self.latest_offset_x
                    current_y = self.latest_offset_y

                message = f"{current_x},{current_y}"

                try:
                    sock.sendto(message.encode('utf-8'), (SERVER_IP, SERVER_PORT))
                    # Loguear envíos puede ser muy verboso, usar debug
                    self.get_logger().debug(f"Posición enviada por UDP: {message}")
                except socket.error as e:
                    # Loguear errores específicos de socket
                    self.get_logger().error(f"Error de socket al enviar UDP: {e}")
                except Exception as e:
                    # Capturar otros posibles errores
                    self.get_logger().error(f"Error inesperado al enviar UDP: {e}")

                # Esperar antes del siguiente envío. Usa el wait del evento para salir rápido.
                # Espera 1 segundo entre envíos (ajusta según necesidad)
                self._stop_event.wait(1.0)

        self.get_logger().info('Hilo de envío UDP finalizado.')

    def destroy_node(self):
        """Limpia recursos al destruir el nodo."""
        self.get_logger().info('Iniciando cierre del nodo procesador...')
        self._stop_event.set() # Señaliza al hilo que debe detenerse
        if hasattr(self, 'sender_thread') and self.sender_thread.is_alive():
             self.get_logger().info('Esperando a que termine el hilo de envío UDP...')
             self.sender_thread.join(timeout=2.0) # Espera máximo 2 segundos
             if self.sender_thread.is_alive():
                  self.get_logger().warn('El hilo de envío UDP no terminó correctamente.')
             else:
                  self.get_logger().info('Hilo de envío UDP terminado.')
        else:
             self.get_logger().info('El hilo de envío UDP no estaba activo o no se inició.')

        cv2.destroyAllWindows() # Cierra ventanas de OpenCV
        self.get_logger().info('Ventanas de OpenCV cerradas.')
        super().destroy_node() # Llama al método de la clase padre
        self.get_logger().info('Nodo procesador destruido.')


def main(args=None):
    rclpy.init(args=args)
    procesador = None # Inicializar a None
    try:
        procesador = ProcesadorImagen()
        print("--> Entrando en el bucle principal (rclpy.spin). Presiona Ctrl+C para salir.")
        rclpy.spin(procesador)
    except KeyboardInterrupt:
        print("\n--> Ctrl+C detectado. Iniciando secuencia de apagado...")
    except Exception as e:
        # Loguear la excepción si el logger está disponible
        if procesador:
            procesador.get_logger().fatal(f"Error crítico durante rclpy.spin(): {e}", exc_info=True)
        else:
            print(f"Error crítico antes de completar la inicialización del nodo: {e}")
    finally:
        print("--> Realizando limpieza final...")
        if procesador:
            procesador.destroy_node()
        else:
             print("--> El nodo no llegó a inicializarse completamente.")
        # Asegurarse de que rclpy se cierre incluso si hubo errores
        if rclpy.ok():
             rclpy.shutdown()
             print("--> rclpy cerrado.")
        print("--> Programa finalizado.")


if __name__ == '__main__':
    main()