COMANDOS PARA EJECUTAR:

Ahora mismo los puertos están configurados para:
Rover -> MAVSDK: 50040 -> PX4: 14541
Dron -> MAVSDK: 50041 -> PX4: 14542


En primer lugar abrimos QGroundControl, y realizamos los siguientes pasos: 


1º Lanzar servidores MAVSDK: 
/home/alvaro/.local/lib/python3.10/site-packages/mavsdk/bin/mavsdk_server -p 50050 udp://:14541
/home/alvaro/.local/lib/python3.10/site-packages/mavsdk/bin/mavsdk_server -p 50051 udp://:14542


2º Lanzar instancia del Rover (Recomendable lanzar primero el rover para que se cargue por defecto del mundo del rover):
PX4_SYS_AUTOSTART=4009 PX4_GZ_MODEL_POSE="1,1" PX4_SIM_PORT=14541 PX4_SIM_MODEL=gz_r1_rover ./build/px4_sitl_default/bin/px4 -i 1


3º Lanzar instancia del Dron:
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="-2,1" PX4_SIM_MODEL=gz_x500_mono_cam_down ./build/px4_sitl_default/bin/px4 -i 2


4º Ejecutar punte de ros_gz para la imagen: 
source ~/ws/install/setup.sh
ros2 run ros_gz_image image_bridge /world/rover/model/x500_mono_cam_down_2/link/camera_link/sensor/imager/image


5º Ya está todo configurado correctamente, para finalizar, activar el env y compilar scripts
source simulacion/bin/activate

#Navegar hasta el directorio correcto;
cd Dron/Imagen/roverDron

#Recomendable lanzar primero el dron, y esperar unos segundos hasta lanzar el rover (para no atropellarlo ;))
python3 tratarImagen.py
python3 moverDron.py
python3 rover.py


