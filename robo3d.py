import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D

# Parámetros del robot
l1 = 1  # Longitud del primer eslabón
l2 = 1  # Longitud del segundo eslabón
l3 = 1  # Longitud del tercer eslabón

# Función para calcular las posiciones de las articulaciones en 3D
def calcular_posiciones(theta1, theta2, theta3):
    x1 = l1 * np.cos(theta1)
    y1 = l1 * np.sin(theta1)
    z1 = 0

    x2 = x1 + l2 * np.cos(theta1 + theta2)
    y2 = y1 + l2 * np.sin(theta1 + theta2)
    z2 = 0

    x3 = x2 + l3 * np.cos(theta1 + theta2) * np.cos(theta3)
    y3 = y2 + l3 * np.sin(theta1 + theta2) * np.cos(theta3)
    z3 = l3 * np.sin(theta3)

    return (x1, y1, z1, x2, y2, z2, x3, y3, z3)

# Crear la figura y los ejes 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(left=0.1, bottom=0.3)
ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)
ax.set_zlim(-3, 3)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Simulación Interactiva de Robot 3 GDL en 3D')

# Graficar el robot inicialmente
(theta1, theta2, theta3) = (np.pi / 4, np.pi / 4, np.pi / 4)
(x1, y1, z1, x2, y2, z2, x3, y3, z3) = calcular_posiciones(theta1, theta2, theta3)
line, = ax.plot([0, x1, x2, x3], [0, y1, y2, y3], [0, z1, z2, z3], 'ro-')

# Crear sliders para los ángulos
ax_theta1 = plt.axes([0.1, 0.1, 0.8, 0.03])
ax_theta2 = plt.axes([0.1, 0.15, 0.8, 0.03])
ax_theta3 = plt.axes([0.1, 0.2, 0.8, 0.03])

slider_theta1 = Slider(ax_theta1, 'Theta1', 0, 2 * np.pi, valinit=theta1)
slider_theta2 = Slider(ax_theta2, 'Theta2', 0, 2 * np.pi, valinit=theta2)
slider_theta3 = Slider(ax_theta3, 'Theta3', 0, 2 * np.pi, valinit=theta3)

# Función de actualización cuando los sliders cambian
def update(val):
    theta1 = slider_theta1.val
    theta2 = slider_theta2.val
    theta3 = slider_theta3.val
    (x1, y1, z1, x2, y2, z2, x3, y3, z3) = calcular_posiciones(theta1, theta2, theta3)
    line.set_data_3d([0, x1, x2, x3], [0, y1, y2, y3], [0, z1, z2, z3])
    fig.canvas.draw_idle()

# Vincular la actualización a los sliders
slider_theta1.on_changed(update)
slider_theta2.on_changed(update)
slider_theta3.on_changed(update)

plt.show()
