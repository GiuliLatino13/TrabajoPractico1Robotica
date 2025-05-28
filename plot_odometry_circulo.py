import numpy as np
import matplotlib.pyplot as plt
from matplotlib import gridspec

def load_and_process_data(filename):
    """Carga y procesa los datos del archivo de odometría"""
    try:
        # Cargar datos desde el archivo
        data = np.loadtxt(filename)
        
        # Eliminar filas con todos los valores cero (robot detenido)
        data = data[~np.all(data == 0, axis=1)]
        
        # Extraer columnas
        timestamps = data[:, 0]  # Tiempo (timestamp)
        x = data[:, 1]          # Posición x
        y = data[:, 2]          # Posición y
        orientation = data[:, 3] # Orientación (yaw)
        
        # Ajustar timestamps para que comiencen en cero
        timestamps = timestamps - timestamps[0]
        
        return timestamps, x, y, orientation
    
    except Exception as e:
        print(f"Error al cargar el archivo: {e}")
        return None, None, None, None

def plot_combined_figure(timestamps, x, y, orientation):
    """Crea una figura combinada con camino y trayectorias"""
    plt.figure(figsize=(14, 10))
    
    # Configurar el grid para los subplots (3 filas, 2 columnas)
    gs = gridspec.GridSpec(3, 2, height_ratios=[3, 1, 1])
    
    # Gráfico del camino (ocupa las dos columnas de la primera fila)
    ax1 = plt.subplot(gs[0, :])
    ax1.plot(x, y, 'b-', linewidth=2)
    ax1.set_title('Camino seguido por el robot')
    ax1.set_xlabel('Posición X (m)')
    ax1.set_ylabel('Posición Y (m)')
    ax1.grid(True)
    ax1.axis('equal')
    
    # Calcular límites comunes para X e Y
    y_min = min(min(x), min(y))
    y_max = max(max(x), max(y))
    y_margin = (y_max - y_min) * 0.1  # 10% de margen
    
    # Gráfico de posición X (segunda fila, primera columna)
    ax2 = plt.subplot(gs[1, 0])
    ax2.plot(timestamps, x, 'r-')
    ax2.set_title('Posición X vs tiempo')
    ax2.set_ylabel('X (m)')
    ax2.set_ylim(y_min - y_margin, y_max + y_margin)
    ax2.grid(True)
    
    # Gráfico de posición Y (segunda fila, segunda columna)
    ax3 = plt.subplot(gs[1, 1])
    ax3.plot(timestamps, y, 'g-')
    ax3.set_title('Posición Y vs tiempo')
    ax3.set_ylabel('Y (m)')
    ax3.set_ylim(y_min - y_margin, y_max + y_margin)
    ax3.grid(True)
    
    # Gráfico de orientación (tercera fila, ocupa ambas columnas)
    ax4 = plt.subplot(gs[2, :])
    ax4.plot(timestamps, orientation, 'm-')
    ax4.set_title('Orientación vs tiempo')
    ax4.set_xlabel('Tiempo (s)')
    ax4.set_ylabel('Orientación (rad)')
    ax4.grid(True)
    
    plt.tight_layout()
    plt.show()

def main():
    # Nombre del archivo de log
    log_file = 'odom_circulo.txt'
    
    # Cargar y procesar datos
    timestamps, x, y, orientation = load_and_process_data(log_file)
    
    if timestamps is not None:
        # Graficar figura combinada
        plot_combined_figure(timestamps, x, y, orientation)
    else:
        print("No se pudieron cargar los datos. Verifica el archivo.")

if __name__ == '__main__':
    main()
