#!/bin/bash
#chmod +x crearServidores.sh

# Verificar si se proporcionó un número como argumento
if [ -z "$1" ]; then
    echo "Uso: $0 <num_instancias>"
    exit 1
fi

NUM_INSTANCIAS=$1
PORT_BASE=50040
UDP_BASE=14541

for ((i=0; i<NUM_INSTANCIAS; i++)); do
    PORT=$((PORT_BASE + i))
    UDP=$((UDP_BASE + i))
    /home/alvaro/.local/lib/python3.10/site-packages/mavsdk/bin/mavsdk_server -p $PORT udp://:$UDP &
    echo "Lanzada instancia en puerto TCP $PORT y UDP $UDP"
done

echo "Todas las instancias han sido lanzadas."


