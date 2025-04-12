#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import time

class NodoCamaraReal(Node):
    def __init__(self):
        # Nombre del nodo
        super().__init__('nodo_camara_real')

        # --- Parámetros Configurables ---
        # Índice de la cámara (0 suele ser la integrada, 1+ o 2+ las USB)
        self.declare_parameter('indice_camara', 0)
        indice_camara = self.get_parameter('indice_camara').get_parameter_value().integer_value

        # Nombre del topic donde se publicará la imagen
        self.declare_parameter('topic_imagen', '/camara_real/image_raw')
        topic_imagen = self.get_parameter('topic_imagen').get_parameter_value().string_value

        # Frame ID para la cabecera del mensaje (puede ser útil si usas TF)
        self.declare_parameter('frame_id_camara', 'camera_link_real')
        self.frame_id_camara = self.get_parameter('frame_id_camara').get_parameter_value().string_value

        # Frecuencia de publicación (en Hz)
        self.declare_parameter('frecuencia', 30.0)
        frecuencia = self.get_parameter('frecuencia').get_parameter_value().double_value
        periodo_temporizador = 1.0 / frecuencia
        # ---------------------------------

        self.get_logger().info(f"Intentando abrir cámara con índice: {indice_camara}")
        self.captura_video = cv2.VideoCapture(indice_camara)

        if not self.captura_video.isOpened():
            self.get_logger().error(f"No se pudo abrir la cámara con índice {indice_camara}")
            # Intenta forzar la destrucción del nodo si la cámara falla
            rclpy.shutdown()
            return # Salir si no se puede abrir

        self.get_logger().info(f"Cámara {indice_camara} abierta correctamente.")
        self.get_logger().info(f"Publicando imágenes en el topic: '{topic_imagen}'")
        self.get_logger().info(f"Frecuencia de publicación: {frecuencia} Hz")

        # Crear el publicador
        self.publicador_imagen = self.create_publisher(Image, topic_imagen, 10)

        # Crear el puente entre OpenCV y ROS
        self.puente_cv = CvBridge()

        # Crear un temporizador para publicar a la frecuencia deseada
        self.temporizador = self.create_timer(periodo_temporizador, self.publicar_frame)

    def publicar_frame(self):
        # Capturar un frame de la cámara
        exito, frame_cv = self.captura_video.read()

        if exito:
            # Convertir el frame de OpenCV a un mensaje de imagen ROS
            # Asegúrate de que la codificación ('bgr8') es la correcta para tu cámara/uso
            mensaje_imagen = self.puente_cv.cv2_to_imgmsg(frame_cv, encoding="bgr8")

            # Añadir la estampa de tiempo actual y el frame_id
            mensaje_imagen.header.stamp = self.get_clock().now().to_msg()
            mensaje_imagen.header.frame_id = self.frame_id_camara

            # Publicar el mensaje
            self.publicador_imagen.publish(mensaje_imagen)
            # self.get_logger().info('Publicando frame...', throttle_duration_sec=1.0) # Loguea info cada segundo
        else:
            self.get_logger().warn('No se pudo capturar frame de la cámara.')

    def destroy_node(self):
        # Asegurarse de liberar la cámara al cerrar
        if hasattr(self, 'captura_video') and self.captura_video.isOpened():
             self.get_logger().info("Liberando la cámara...")
             self.captura_video.release()
             self.get_logger().info("Cámara liberada.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    nodo_camara = None
    try:
        nodo_camara = NodoCamaraReal()
        # Solo entra en spin si la cámara se abrió correctamente en __init__
        if rclpy.ok():
             rclpy.spin(nodo_camara)
    except KeyboardInterrupt:
        print("Cerrando nodo de cámara por interrupción de teclado.")
    except Exception as e:
        if nodo_camara:
            nodo_camara.get_logger().error(f"Error inesperado: {e}")
        else:
            print(f"Error durante la inicialización del nodo: {e}")
    finally:
        # Asegurarse de que el nodo se destruye correctamente
        if nodo_camara and rclpy.ok():
            nodo_camara.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Nodo de cámara finalizado.")


if __name__ == '__main__':
    main()
