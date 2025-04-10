#!/bin/bash

#chmod +x crearRovers.sh

# Verificar que se haya proporcionado un número como parámetro
if [ -z "$1" ]; then
  echo "Por favor, proporciona el número de rovers como argumento."
  exit 1
fi

# Número de rovers a lanzar
num_drones=$1

# Verificar que el número de drones sea un valor positivo
if ! [[ "$num_drones" =~ ^[0-9]+$ ]] || [ "$num_drones" -le 0 ]; then
  echo "El número de rovers debe ser un número entero positivo."
  exit 1
fi

# Cambiar al directorio de inicio (/home)
cd /home || { echo "No se pudo acceder al directorio /home"; exit 1; }

# Cambiar al directorio de PX4-Autopilot
cd alvaro/PX4-Autopilot/ || { echo "No se pudo acceder al directorio /alvaro/PX4-Autopilot/"; exit 1; }

# Ejecutar los drones con el parámetro -i incrementando el número
for i in $(seq 1 $num_drones); do
  PX4_SYS_AUTOSTART=4009 PX4_SIM_MODEL=gz_r1_rover ./build/px4_sitl_default/bin/px4 -i $i &
  echo "Iniciando dron con ID $i"
done

echo "$num_drones rovers han sido lanzados."
