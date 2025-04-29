import pandas as pd
import matplotlib.pyplot as plt
import os

# --- Configuración ---
# Nombre del archivo CSV generado por moverDron
nombre_archivo_csv = "trayectoria.csv"

# --- Lectura del Archivo CSV ---
try:
    # Comprobar si el archivo existe
    if not os.path.exists(nombre_archivo_csv):
        print(f"Error: El archivo '{nombre_archivo_csv}' no se encontró en el directorio actual.")
        exit() # Salir si el archivo no existe

    # Leer el archivo CSV usando pandas
    # Asegurarse de que la primera columna se interprete como texto (o datetime si se prefiere, pero no es necesario para plotear X-Y)
    datos = pd.read_csv(nombre_archivo_csv)

    # Comprobar si el DataFrame está vacío o no tiene las columnas esperadas
    if datos.empty:
        print(f"Error: El archivo '{nombre_archivo_csv}' está vacío.")
        exit()
    if not {'North_m', 'East_m', 'Down_m'}.issubset(datos.columns):
         print(f"Error: El archivo '{nombre_archivo_csv}' no contiene las columnas esperadas (North_m, East_m, Down_m).")
         exit()

except Exception as e:
    print(f"Error al leer el archivo CSV '{nombre_archivo_csv}': {e}")
    exit()

# --- Preparación del Gráfico ---
print(f"Datos leídos correctamente. Generando gráfico de '{nombre_archivo_csv}'...")

# Crear figura y ejes para el gráfico
fig, ax = plt.subplots(figsize=(10, 8)) # Tamaño de la figura (ancho, alto) en pulgadas

# --- Generación del Gráfico 2D (Vista Superior) ---
# Graficar Este (Eje X) vs Norte (Eje Y)
ax.plot(datos['East_m'], datos['North_m'], marker='.', linestyle='-', label='Trayectoria NED')

# Marcar el punto de inicio
ax.plot(datos['East_m'].iloc[0], datos['North_m'].iloc[0], 'go', markersize=8, label='Inicio') # 'go' = círculo verde

# Marcar el punto final
ax.plot(datos['East_m'].iloc[-1], datos['North_m'].iloc[-1], 'rx', markersize=10, label='Fin') # 'rx' = X roja

# --- Configuración del Gráfico ---
ax.set_title('Trayectoria del Dron (Vista Superior)') # Título
ax.set_xlabel('Coordenada Este (m)') # Etiqueta eje X
ax.set_ylabel('Coordenada Norte (m)') # Etiqueta eje Y
ax.set_aspect('equal', adjustable='box') # Asegura que las escalas en X e Y sean iguales (importante para trayectorias)
ax.grid(True) # Muestra una cuadrícula
ax.legend() # Muestra la leyenda (Inicio, Fin, Trayectoria)

# --- Mostrar Gráfico ---
print("Gráfico generado. Mostrando ventana...")
plt.show()
print("Ventana del gráfico cerrada.")
