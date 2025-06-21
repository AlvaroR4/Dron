import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray
import time
import datetime
import csv
import traceback

ROS_TOPIC_DATOS_DETECCION_INPUT = '/tello/datos_deteccion' # Desde nodo_procesador_imagen
ROS_TOPIC_COMANDOS_VELOCIDAD_OUTPUT = '/tello/comandos_velocidad' # Hacia nodo_camara_tello

ARCHIVO_PASO_PUERTAS = "paso_puertas_tello_ros2.csv" 
VELOCIDAD_AVANCE_TELLO = 40
VELOCIDAD_LATERAL_TELLO = 30
VELOCIDAD_VERTICAL_TELLO = 30
VELOCIDAD_YAW_TELLO = 0 

# Márgenes de error para alineación (convertidos de offset normalizado a un factor)
# O, más simple, usar los offsets normalizados directamente.
# Por ahora, se usan los offsets normalizados directamente contra umbrales más pequeños.
UMBRAL_OFFSET_X_ALINEADO = 0.09  # Equivalente a ~25px en 640px width (0.08 * 320)
UMBRAL_OFFSET_Y_ALINEADO = 0.09  # Equivalente a ~19px en 480px height (0.08 * 240)

UMBRAL_OFFSET_X_AVANZANDO = 0.02
UMBRAL_OFFSET_Y_AVANZANDO = 0.02

CONTADOR_PERDIDO_MAX = 15       # Ciclos antes de declarar objetivo perdido
DISTANCIA_UMBRAL_CERCA = 1.5    # Metros, para considerar que estamos "cerca" de la puerta
UMBRAL_AUMENTO_DIST = 0.5       # Metros, aumento de distancia para confirmar paso
TIEMPO_AVANCE_EXTRA_SEGS = 4.0  # Segundos de avance extra tras pasar puerta
CONTADOR_BUSQUEDA_MAX = 120     # Ciclos de búsqueda antes de rendirse (aprox. 120 * (1/frec_control) segs)

# Estados de la Misión
ESTADO_INICIO = 0
ESTADO_BUSCANDO = 1
ESTADO_ALINEANDO = 2
ESTADO_AVANZANDO = 3
ESTADO_AVANCE_EXTRA = 4
ESTADO_MISION_COMPLETA = 5 
ESTADO_ERROR = 6

estado_nombres = {
    ESTADO_INICIO: "INICIO", ESTADO_BUSCANDO: "BUSCANDO_PUERTA",
    ESTADO_ALINEANDO: "ALINEANDO_PUERTA", ESTADO_AVANZANDO: "AVANZANDO_HACIA_PUERTA",
    ESTADO_AVANCE_EXTRA: "AVANCE_EXTRA_POST_PASO",
    ESTADO_MISION_COMPLETA: "MISION_COMPLETADA", ESTADO_ERROR: "ERROR_EN_MISION"
}

class NodoControlLogica(Node):
    def __init__(self):
        super().__init__('nodo_control_logica')
        self.get_logger().info("Iniciando Nodo de Control y Lógica de Misión.")

        # Estado inicial y variables de lógica
        self.estado_actual = ESTADO_INICIO
        self.objetivo_perdido_contador = 0
        self.min_distancia_vista_actual_puerta = float('inf')
        self.fase_avance_actual_puerta = 'INICIAL' # 'INICIAL', 'CERCA'
        self.puertas_pasadas_contador = 0
        self.ciclos_sin_objetivo_buscando = 0
        self.tiempo_inicio_avance_extra = 0.0

        # Suscriptor para los datos de detección
        qos_profile_sub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, # Esperamos datos fiables
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.suscripcion_datos_deteccion = self.create_subscription(
            Float32MultiArray,
            ROS_TOPIC_DATOS_DETECCION_INPUT,
            self.callback_control_logica,
            qos_profile_sub
        )
        self.get_logger().info(f"Suscrito a datos de detección en: {ROS_TOPIC_DATOS_DETECCION_INPUT}")

        # Publicador para los comandos de velocidad
        qos_profile_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.publicador_comandos_velocidad = self.create_publisher(
            Float32MultiArray, ROS_TOPIC_COMANDOS_VELOCIDAD_OUTPUT, qos_profile_pub)
        self.get_logger().info(f"Publicando comandos de velocidad en: {ROS_TOPIC_COMANDOS_VELOCIDAD_OUTPUT}")

        # Inicializar archivo CSV
        try:
            self.get_logger().info(f"Inicializando archivo CSV: {ARCHIVO_PASO_PUERTAS}")
            cabecera = ["Timestamp", "NumeroPuerta", "EstadoAlPasar", "MinDistDetectada"]
            with open(ARCHIVO_PASO_PUERTAS, mode='w', newline='', encoding='utf-8') as file:
                writer = csv.writer(file)
                writer.writerow(cabecera)
        except Exception as e_csv:
            self.get_logger().error(f"No se pudo inicializar el archivo CSV: {e_csv}")
        
        # Transición inicial de estado
        self.cambiar_estado(ESTADO_BUSCANDO) # Empezar a buscar directamente


    def escribir_log_csv(self, datos_fila):
        try:
            with open(ARCHIVO_PASO_PUERTAS, mode='a', newline='', encoding='utf-8') as file_object:
                writer = csv.writer(file_object)
                writer.writerow(datos_fila)
        except Exception as e_csv_write:
            self.get_logger().warn(f"Error escribiendo en CSV: {e_csv_write}")

    def cambiar_estado(self, nuevo_estado):
        estado_actual_nombre = estado_nombres.get(self.estado_actual, f"DESCONOCIDO({self.estado_actual})")
        nuevo_estado_nombre = estado_nombres.get(nuevo_estado, f"DESCONOCIDO({nuevo_estado})")

        if self.estado_actual != nuevo_estado:
            self.get_logger().info(f"CAMBIO ESTADO: {estado_actual_nombre} -> {nuevo_estado_nombre}")
            self.estado_actual = nuevo_estado
            self.objetivo_perdido_contador = 0
            self.ciclos_sin_objetivo_buscando = 0 

            if nuevo_estado == ESTADO_ALINEANDO or nuevo_estado == ESTADO_BUSCANDO:
                self.min_distancia_vista_actual_puerta = float('inf')
                self.fase_avance_actual_puerta = 'INICIAL'

            if nuevo_estado in [ESTADO_BUSCANDO, ESTADO_MISION_COMPLETA, ESTADO_ERROR]:
                self.publicar_velocidades(0, 0, 0, 0)


    def publicar_velocidades(self, lr, fb, ud, yv):
        msg_vel = Float32MultiArray()
        msg_vel.data = [float(lr), float(fb), float(ud), float(yv)]
        self.publicador_comandos_velocidad.publish(msg_vel)
        # self.get_logger().debug(f"Publicando velocidades: lr={lr}, fb={fb}, ud={ud}, yv={yv}")


    def callback_control_logica(self, msg_datos):
        # Extraer datos: [offset_x_norm, offset_y_norm, distancia_m, num_targets]
        if len(msg_datos.data) < 4:
            self.get_logger().warn("Mensaje de datos de detección incompleto.", throttle_duration_sec=5.0)
            return
        
        offset_x_norm = msg_datos.data[0]
        offset_y_norm = msg_datos.data[1] # Positivo es ABAJO en imagen
        distancia_m = msg_datos.data[2]
        num_targets = int(msg_datos.data[3])

        # Inicializar comandos de velocidad
        lr, fb, ud, yv = 0, 0, 0, 0
        objetivo_detectado_valido = num_targets > 0 and distancia_m > 0 and distancia_m != float('inf')

        if self.estado_actual == ESTADO_BUSCANDO:
            if objetivo_detectado_valido:
                self.get_logger().info(f"¡Objetivo encontrado! (Puerta #{self.puertas_pasadas_contador + 1}) Dist: {distancia_m:.2f}m")
                self.ciclos_sin_objetivo_buscando = 0
                self.cambiar_estado(ESTADO_ALINEANDO)
            else:
                self.ciclos_sin_objetivo_buscando += 1
                # self.get_logger().info(f"Buscando siguiente puerta... (Ciclo sin objetivo: {self.ciclos_sin_objetivo_buscando}/{CONTADOR_BUSQUEDA_MAX})", throttle_duration_sec=2.0)
                if self.ciclos_sin_objetivo_buscando % 30 == 0 : # Log every ~1 second if 30Hz
                    self.get_logger().info(f"Buscando siguiente puerta... (Ciclo {self.ciclos_sin_objetivo_buscando}/{CONTADOR_BUSQUEDA_MAX})")


                if self.ciclos_sin_objetivo_buscando >= CONTADOR_BUSQUEDA_MAX:
                    if self.puertas_pasadas_contador > 0:
                        self.get_logger().info("No se encontraron más puertas tras búsqueda. Misión completada.")
                        self.cambiar_estado(ESTADO_MISION_COMPLETA)
                    else:
                        self.get_logger().warn("No se encontró ni la primera puerta tras búsqueda. Revisar entorno/detección.")
                        self.cambiar_estado(ESTADO_ERROR) 
                        
                # AQUÍ SE PODRÍA IMPLEMENTAR COSAS COMO GIRAR EL DRON PARA BUSCAR
                lr, fb, ud, yv = 0, 0, 0, VELOCIDAD_YAW_TELLO // 3 # Pequeño giro para buscar

        elif self.estado_actual == ESTADO_ALINEANDO:
            self.ciclos_sin_objetivo_buscando = 0
            if not objetivo_detectado_valido:
                self.objetivo_perdido_contador += 1
                self.get_logger().warn(f"Objetivo perdido durante ALINEACIÓN ({self.objetivo_perdido_contador}/{CONTADOR_PERDIDO_MAX})", throttle_duration_sec=1.0)
                if self.objetivo_perdido_contador >= CONTADOR_PERDIDO_MAX:
                    self.get_logger().warn("Objetivo perdido definitivamente en ALINEACIÓN. Volviendo a BUSCAR.")
                    self.cambiar_estado(ESTADO_BUSCANDO)
                lr, fb, ud, yv = 0, 0, 0, 0 # Detenerse si se pierde
            else:
                self.objetivo_perdido_contador = 0
                # offset_x_norm: negativo=izq, positivo=der
                # offset_y_norm: negativo=arriba, positivo=abajo (en imagen)
                
                # Para el dron:
                # lr: negativo=izquierda, positivo=derecha
                # ud: negativo=abajo, positivo=arriba

                alineado_x = abs(offset_x_norm) <= UMBRAL_OFFSET_X_ALINEADO
                alineado_y = abs(offset_y_norm) <= UMBRAL_OFFSET_Y_ALINEADO

                if not alineado_x:
                    # Si offset_x_norm < 0 (obj a la izq), dron debe ir a la izq (lr negativo)
                    lr = -VELOCIDAD_LATERAL_TELLO if offset_x_norm < 0 else VELOCIDAD_LATERAL_TELLO
                
                if not alineado_y:
                    # Si offset_y_norm < 0 (obj arriba en imagen), dron debe subir (ud pos)
                    # Si offset_y_norm > 0 (obj abajo en imagen), dron debe bajar (ud neg)
                    ud = VELOCIDAD_VERTICAL_TELLO if offset_y_norm < 0 else -VELOCIDAD_VERTICAL_TELLO
                
                self.get_logger().info(f"Alineando: dx={offset_x_norm:.2f} (lr={lr}), dy={offset_y_norm:.2f} (ud={ud}), Dist={distancia_m:.2f}m", throttle_duration_sec=0.5)

                if alineado_x and alineado_y:
                    self.get_logger().info("Alineado con la puerta. Iniciando AVANCE.")
                    self.cambiar_estado(ESTADO_AVANZANDO)
                    lr, fb, ud, yv = 0, 0, 0, 0 # Comando de transición, el avance se aplica en el siguiente ciclo

        elif self.estado_actual == ESTADO_AVANZANDO:
            self.ciclos_sin_objetivo_buscando = 0
            puerta_pasada_confirmada = False

            if objetivo_detectado_valido:
                self.objetivo_perdido_contador = 0
                self.min_distancia_vista_actual_puerta = min(self.min_distancia_vista_actual_puerta, distancia_m)
                self.get_logger().info(f"Avanzando: Dist={distancia_m:.2f}m (min={self.min_distancia_vista_actual_puerta:.2f}m), Fase='{self.fase_avance_actual_puerta}'", throttle_duration_sec=0.5)

                if distancia_m < DISTANCIA_UMBRAL_CERCA:
                    if self.fase_avance_actual_puerta == 'INICIAL':
                        self.get_logger().info(f"--- Fase Avance: CERCA (Dist: {distancia_m:.2f}m < {DISTANCIA_UMBRAL_CERCA}m) ---")
                        self.fase_avance_actual_puerta = 'CERCA'
                
                # Condición de Paso 1: Aumento de distancia después de estar cerca
                if self.fase_avance_actual_puerta == 'CERCA' and distancia_m > self.min_distancia_vista_actual_puerta + UMBRAL_AUMENTO_DIST:
                    self.get_logger().info(f"PASO DE PUERTA (Cond 1: Aumento dist: {distancia_m:.2f}m > min_visto {self.min_distancia_vista_actual_puerta:.2f}m + umbral {UMBRAL_AUMENTO_DIST}m)")
                    puerta_pasada_confirmada = True
                
                if not puerta_pasada_confirmada:
                    fb = VELOCIDAD_AVANCE_TELLO
                    # Correcciones laterales y verticales durante el avance
                    if abs(offset_x_norm) > UMBRAL_OFFSET_X_AVANZANDO:
                        lr = -VELOCIDAD_LATERAL_TELLO if offset_x_norm < 0 else VELOCIDAD_LATERAL_TELLO
                    if abs(offset_y_norm) > UMBRAL_OFFSET_Y_AVANZANDO:
                        ud = VELOCIDAD_VERTICAL_TELLO if offset_y_norm < 0 else -VELOCIDAD_VERTICAL_TELLO
            else: # Objetivo NO detectado mientras avanzábamos
                self.objetivo_perdido_contador += 1
                self.get_logger().warn(f"Objetivo no detectado AVANZANDO ({self.objetivo_perdido_contador}/{CONTADOR_PERDIDO_MAX}).", throttle_duration_sec=1.0)
                lr, fb, ud, yv = 0, 0, 0, 0 # Detenerse si se pierde mientras se decide

                if self.objetivo_perdido_contador >= CONTADOR_PERDIDO_MAX:
                    if self.fase_avance_actual_puerta == 'CERCA':
                        self.get_logger().info("PASO DE PUERTA (Cond 2: Pérdida de objetivo tras estar CERCA).")
                        puerta_pasada_confirmada = True
                    else:
                        self.get_logger().warn("Objetivo perdido antes de confirmar cercanía. Asumiendo paso igualmente (podría ser error).")
                        # Podrías decidir volver a BUSCAR aquí si no quieres asumir el paso
                        puerta_pasada_confirmada = True # O False y cambiar a ESTADO_BUSCAR si se prefiere más cautela
            
            if puerta_pasada_confirmada:
                self.puertas_pasadas_contador += 1
                timestamp = datetime.datetime.now().isoformat()
                self.get_logger().info(f"--- PUERTA {self.puertas_pasadas_contador} PASADA! Registrando. ---")
                self.escribir_log_csv([timestamp, self.puertas_pasadas_contador, self.fase_avance_actual_puerta, f"{self.min_distancia_vista_actual_puerta:.2f}"])
                
                self.tiempo_inicio_avance_extra = time.time()
                self.cambiar_estado(ESTADO_AVANCE_EXTRA)
                # El comando de velocidad para avance extra se aplicará en el estado AVANCE_EXTRA
        
        elif self.estado_actual == ESTADO_AVANCE_EXTRA:
            tiempo_transcurrido = time.time() - self.tiempo_inicio_avance_extra
            if tiempo_transcurrido < TIEMPO_AVANCE_EXTRA_SEGS:
                self.get_logger().info(f"En AVANCE_EXTRA ({tiempo_transcurrido:.1f}/{TIEMPO_AVANCE_EXTRA_SEGS:.1f}s)...", throttle_duration_sec=0.5)
                fb = VELOCIDAD_AVANCE_TELLO # Continuar avanzando
                # No se hacen correcciones aquí, es un avance "a ciegas" corto
            else:
                self.get_logger().info("Avance extra completado. Volviendo a BUSCAR.")
                self.cambiar_estado(ESTADO_BUSCANDO)
                fb = 0 # Asegurar parada antes de buscar
        
        elif self.estado_actual == ESTADO_MISION_COMPLETA:
            self.get_logger().info("Misión completada. Dron debería estar detenido. Esperando cierre manual.", throttle_duration_sec=5.0)
            lr, fb, ud, yv = 0, 0, 0, 0 # Asegurar que está detenido

        elif self.estado_actual == ESTADO_ERROR:
            self.get_logger().error("Estado de ERROR alcanzado. Dron debería detenerse. Revisar logs.", throttle_duration_sec=5.0)
            lr, fb, ud, yv = 0, 0, 0, 0 # Asegurar que está detenido

        # Publicar comandos de velocidad calculados (excepto si ya se hizo en cambiar_estado para parada)
        if not (self.estado_actual in [ESTADO_BUSCANDO, ESTADO_MISION_COMPLETA, ESTADO_ERROR] and lr==0 and fb==0 and ud==0 and yv==0):
             self.publicar_velocidades(lr, fb, ud, yv)


    def destroy_node(self):
        self.get_logger().info("Destruyendo NodoControlLogica...")
        # Asegurar que se envía un último comando de parada si el nodo se cierra inesperadamente
        self.publicar_velocidades(0,0,0,0)
        self.get_logger().info("Último comando de parada enviado.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    nodo_control = None
    try:
        nodo_control = NodoControlLogica()
        rclpy.spin(nodo_control)
    except KeyboardInterrupt:
        if nodo_control:
            nodo_control.get_logger().info("Ctrl+C detectado, cerrando NodoControlLogica.")
        else:
            print("Ctrl+C detectado antes de inicializar NodoControlLogica.")
    except Exception as e_main:
        if nodo_control:
            nodo_control.get_logger().fatal(f"Error inesperado en main de NodoControlLogica: {e_main}")
            nodo_control.get_logger().fatal(traceback.format_exc())
        else:
            print(f"Error inesperado en main antes de inicializar NodoControlLogica: {e_main}")
            print(traceback.format_exc())
    finally:
        if nodo_control:
            nodo_control.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Programa NodoControlLogica finalizado.")

if __name__ == '__main__':
    main()