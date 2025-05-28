import numpy as np
import matplotlib.pyplot as plt
from matplotlib import gridspec

def load_and_process_data(filename):
    # Cargar datos desde el archivo
    data = np.loadtxt(filename)
    
    # Eliminar filas con todos los valores cero
    data = data[~np.all(data == 0, axis=1)]
    
    # Extraer columnas
    timestamps = data[:, 0]
    x = data[:, 1]
    y = data[:, 2]
    orientation = data[:, 3]
    linear_vel = data[:, 4]
    angular_vel = data[:, 5]
    
    # Ajustar timestamps para que comiencen en cero
    timestamps = timestamps - timestamps[0]
    
    return timestamps, x, y, orientation, linear_vel, angular_vel

def plot_robot_path(x, y):
    plt.figure(figsize=(8, 8))
    plt.plot(x, y, 'b-', linewidth=2)
    plt.title('Camino seguido por el robot')
    plt.xlabel('Posición X (m)')
    plt.ylabel('Posición Y (m)')
    plt.grid(True)
    plt.axis('equal')  # Relación de aspecto 1:1
    plt.show()

def plot_pose_vs_time(timestamps, x, y, orientation):
    plt.figure(figsize=(12, 8))
    
    plt.subplot(3, 1, 1)
    plt.plot(timestamps, x, 'r-')
    plt.title('Trayectoria del robot vs tiempo')
    plt.ylabel('Posición X (m)')
    plt.grid(True)
    
    plt.subplot(3, 1, 2)
    plt.plot(timestamps, y, 'g-')
    plt.ylabel('Posición Y (m)')
    plt.grid(True)
    
    plt.subplot(3, 1, 3)
    plt.plot(timestamps, orientation, 'b-')
    plt.ylabel('Orientación (rad)')
    plt.xlabel('Tiempo (s)')
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()

def plot_velocity_vs_time(timestamps, linear_vel, angular_vel):
    plt.figure(figsize=(12, 6))
    
    plt.subplot(2, 1, 1)
    plt.plot(timestamps, linear_vel, 'm-')
    plt.title('Velocidad del robot vs tiempo')
    plt.ylabel('Velocidad Lineal (m/s)')
    plt.grid(True)
    
    plt.subplot(2, 1, 2)
    plt.plot(timestamps, angular_vel, 'c-')
    plt.ylabel('Velocidad Angular (rad/s)')
    plt.xlabel('Tiempo (s)')
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()

def main():
    # Nombre del archivo de log
    log_file = 'odom_log.txt'
    
    # Cargar y procesar datos
    timestamps, x, y, orientation, linear_vel, angular_vel = load_and_process_data(log_file)
    
    # Generar gráficos
    plot_robot_path(x, y)
    plot_pose_vs_time(timestamps, x, y, orientation)
    plot_velocity_vs_time(timestamps, linear_vel, angular_vel)

if __name__ == '__main__':
    main()
