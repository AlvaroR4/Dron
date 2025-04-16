import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

TAMANO_REAL_PUERTA_M = 1.5 # Ejemplo: Ancho REAL de la puerta_grande_1 del puertas_rojas.sdf
"""
Calcular desde FOV: No tenemos <fx> o <fy> directamente. Usamos la fórmula con el FOV horizontal (HFOV) y el ancho (width) para obtener fx. Asumiremos fy = fx.

    fx = (width / 2) / tan(HFOV / 2)
    width = 1280 píxeles
    HFOV = 1.74 radianes
    fx = (1280 / 2) / tan(1.74 / 2) = 640 / tan(0.87)
    tan(0.87) ≈ 1.1866
    fx ≈ 640 / 1.1866 ≈ 539.35 píxeles.

Valor: Usaremos fy = fx. DISTANCIA_FOCAL_PIXELS = 539.35 (aproximadamente).
"""
DISTANCIA_FOCAL_PIXELS = 539.35 

class ImageProcessorDistanceEst(Node):
    def __init__(self):
        super().__init__('image_processor_distance_estimator')

        self.subscription = self.create_subscription(
            Image,
            '/world/puertas_rojas/model/x500_mono_cam_0/link/camera_link/sensor/imager/image',
            self.listener_callback,
            10)
        self.br = CvBridge()
        self.get_logger().info(f"Estimador de distancia visual iniciado.")
        self.get_logger().info(f"Usando Ancho Real={TAMANO_REAL_PUERTA_M}m, Focal={DISTANCIA_FOCAL_PIXELS}px")
        self.get_logger().warn("Verifica que el rango de color detecta ROJO correctamente.")

    def listener_callback(self, data):
        self.get_logger().info('Recibiendo frame...')
        try:
            current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error CvBridge: {e}")
            return

        img_display = current_frame.copy()
        height, width, _ = img_display.shape
        center_x = width // 2
        center_y = height // 2

        lower_red = np.array([0, 0, 100])
        upper_red = np.array([80, 80, 255])

        mask = cv2.inRange(current_frame, lower_red, upper_red)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        found_target = False
        offset_x = 0
        offset_y = 0
        tamanio_ancho_pixels = 0.0
        distancia_estimada = 0.0

        if contours:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            current_area = cv2.contourArea(c)
            if current_area > 100:
                found_target = True
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # --- Cambio Principal: Usar minAreaRect ---
                rect = cv2.minAreaRect(c)
                # rect devuelve ((centro_x, centro_y), (ancho, alto), angulo_rotacion)
                (w_rot, h_rot) = rect[1] # Extraer dimensiones del rect. rotado

                # Usamos la dimensión MENOR como el ancho, ya que AnchoReal(1.5) < AltoReal(2.0)
                tamanio_ancho_pixels = float(min(w_rot, h_rot))

                # Dibujar el rectángulo rotado (opcional, para verificar)
                box_points = cv2.boxPoints(rect) # Obtiene las 4 esquinas
                box_points = np.intp(box_points) # Convierte a enteros
                cv2.drawContours(img_display,[box_points],0,(255,0,255),2) # Dibujar en magenta
                # -----------------------------------------

                if tamanio_ancho_pixels > 1:
                    distancia_estimada = (TAMANO_REAL_PUERTA_M * DISTANCIA_FOCAL_PIXELS) / tamanio_ancho_pixels

                cv2.circle(img_display, (cX, cY), 7, (255, 255, 255), -1)
                # cv2.drawContours(img_display, [c], -1, (0, 255, 0), 3) # Contorno original verde
                # cv2.rectangle(img_display,(x_rect,y_rect),(x_rect+w_rect,y_rect+h_rect),(255,0,0),2) # Ya no usamos este

                offset_x = cX - center_x
                offset_y = cY - center_y

                log_msg = (f"Target: YES | Offset:({offset_x},{offset_y}) | "
                           f"W_px(rot):{tamanio_ancho_pixels:.0f} | Dist Est:{distancia_estimada:.2f}m")
                self.get_logger().info(log_msg)

                cv2.putText(img_display, f"x:{offset_x},y:{offset_y}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(img_display, f"W(rot):{tamanio_ancho_pixels:.0f}px", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2) # Mostrar W rotada
                cv2.putText(img_display, f"DistEst:{distancia_estimada:.2f}m", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if not found_target:
             self.get_logger().info('Target: NO')

        cv2.imshow("Estimador Distancia Visual (Ancho Rotado)", img_display) # Título actualizado
        cv2.waitKey(1)


    def destroy_node(self):
        self.get_logger().info('Limpiando nodo estimador...')
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    estimator_node = None
    try:
        estimator_node = ImageProcessorDistanceEst()
        rclpy.spin(estimator_node)
    except KeyboardInterrupt:
        print("--> Ctrl+C detectado...")
    except Exception as e:
        if estimator_node:
             estimator_node.get_logger().fatal(f"Error crítico: {e}", exc_info=True)
        else:
             print(f"Error crítico antes de inicializar: {e}")
    finally:
        print("--> Limpiando nodo estimador...")
        if estimator_node:
             estimator_node.destroy_node()
        if rclpy.ok():
             rclpy.shutdown()
        print("--> Nodo estimador finalizado.")

if __name__ == '__main__':
    main()
