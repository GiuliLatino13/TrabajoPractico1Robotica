import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

def graficar_camino(archivo='cuadrado.txt'):
    """
    Grafica el camino X vs Y desde un archivo de datos de odometría
    Args:
        archivo (str): Nombre del archivo con los datos (formato: timestamp x y orientacion vel_lineal vel_angular)
    """
    try:
        # Cargar datos, omitiendo filas con ceros
        datos = np.loadtxt(archivo)
        datos = datos[~np.all(datos == 0, axis=1)]
        
        # Extraer coordenadas X, Y
        x = datos[:, 1]
        y = datos[:, 2]
        
        # Configurar gráfico
        fig, ax = plt.subplots(figsize=(10, 10))
        
        # Graficar camino con flecha indicando dirección
        line = ax.plot(x, y, 'b-', linewidth=2, label='Camino del robot')[0]
        add_arrow(line, position=x[-1]/len(x))  # Flecha al final
        
        # Añadir punto de inicio y final
        ax.plot(x[0], y[0], 'go', markersize=10, label='Inicio')
        ax.plot(x[-1], y[-1], 'ro', markersize=10, label='Final')
        
        # Configuraciones del gráfico
        ax.set_title('Camino del Robot (X vs Y)\nArchivo: '+archivo, pad=20)
        ax.set_xlabel('Posición X (metros)')
        ax.set_ylabel('Posición Y (metros)')
        ax.grid(True, which='both', linestyle='--', alpha=0.7)
        ax.axis('equal')  # Relación de aspecto 1:1
        ax.legend(loc='upper right')
        
        # Ajustar marcas de los ejes cada 0.5 metros
        ax.xaxis.set_major_locator(MultipleLocator(0.5))
        ax.yaxis.set_major_locator(MultipleLocator(0.5))
        
        # Mostrar distancia total recorrida
        distancia = calcular_distancia(x, y)
        ax.text(0.02, 0.98, f'Distancia total: {distancia:.2f} m', 
                transform=ax.transAxes, verticalalignment='top',
                bbox=dict(facecolor='white', alpha=0.8))
        
        plt.tight_layout()
        plt.show()
        
    except Exception as e:
        print(f"Error al procesar el archivo: {e}")

def add_arrow(line, position=None, direction='right', size=15, color=None):
    """Añade una flecha a una línea en el gráfico"""
    if color is None:
        color = line.get_color()
        
    xdata = line.get_xdata()
    ydata = line.get_ydata()
    
    if position is None:
        position = xdata.mean()
        
    start_ind = np.argmin(np.absolute(xdata - position))
    if direction == 'right':
        end_ind = start_ind + 1
    else:
        end_ind = start_ind - 1
        
    ax = line.axes
    ax.annotate('',
        xytext=(xdata[start_ind], ydata[start_ind]),
        xy=(xdata[end_ind], ydata[end_ind]),
        arrowprops=dict(arrowstyle="->", color=color),
        size=size
    )

def calcular_distancia(x, y):
    """Calcula la distancia total recorrida sumando segmentos"""
    return np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2))

if __name__ == '__main__':
    graficar_camino('4vel.txt')
