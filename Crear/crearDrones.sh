#!/bin/bash

# Verificar que se haya proporcionado un número como parámetro
if [ -z "$1" ]; then
  echo "Por favor, proporciona el número de drones como argumento."
  exit 1
fi

# Número de drones a lanzar
num_drones=$1

# Verificar que el número de drones sea un valor positivo
if ! [[ "$num_drones" =~ ^[0-9]+$ ]] || [ "$num_drones" -le 0 ]; then
  echo "El número de drones debe ser un número entero positivo."
  exit 1
fi

# Cambiar al directorio de inicio (/home)
cd /home || { echo "No se pudo acceder al directorio /home"; exit 1; }

# Cambiar al directorio de PX4-Autopilot
cd alvaro/PX4-Autopilot/ || { echo "No se pudo acceder al directorio /alvaro/PX4-Autopilot/"; exit 1; }

# Lanzar el primer dron en una nueva terminal (esperando 15 segundos después)
xterm -e "PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1; bash" &
echo "Iniciando el primer dron con ID 1"
sleep 15  # Esperar 15 segundos antes de continuar con el resto

# Ejecutar el resto de los drones en nuevas terminales
for i in $(seq 2 $num_drones); do
  xterm -e "PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i $i; bash" &
  echo "Iniciando dron con ID $i"
done

echo "$num_drones drones han sido lanzados."
