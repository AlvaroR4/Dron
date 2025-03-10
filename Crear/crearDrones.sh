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

# Ejecutar los drones con el parámetro -i incrementando el número
for i in $(seq 1 $num_drones); do
  PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i $i &
  echo "Iniciando dron con ID $i"
done

echo "$num_drones drones han sido lanzados."

