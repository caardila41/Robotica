import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Parámetros del robot
l1 = 1  # Longitud del primer eslabón
l2 = 1  # Longitud del segundo eslabón

# Función para calcular las posiciones de las articulaciones
def calcular_posiciones(theta1, theta2):
    x1 = l1 * np.cos(theta1)
    y1 = l1 * np.sin(theta1)
    x2 = x1 + l2 * np.cos(theta1 + theta2)
    y2 = y1 + l2 * np.sin(theta1 + theta2)
    return (x1, y1, x2, y2)

# Crear la figura y los ejes
fig, ax = plt.subplots()
plt.subplots_adjust(left=0.1, bottom=0.3)
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('Simulación Interactiva de Robot 2 GDL')

# Graficar el robot inicialmente
(theta1, theta2) = (np.pi / 4, np.pi / 4)
(x1, y1, x2, y2) = calcular_posiciones(theta1, theta2)
line, = ax.plot([0, x1, x2], [0, y1, y2], 'ro-')

# Crear sliders para los ángulos
ax_theta1 = plt.axes([0.1, 0.1, 0.8, 0.03])
ax_theta2 = plt.axes([0.1, 0.15, 0.8, 0.03])
slider_theta1 = Slider(ax_theta1, 'Theta1', 0, 2 * np.pi, valinit=theta1)
slider_theta2 = Slider(ax_theta2, 'Theta2', 0, 2 * np.pi, valinit=theta2)

# Función de actualización cuando los sliders cambian
def update(val):
    theta1 = slider_theta1.val
    theta2 = slider_theta2.val
    (x1, y1, x2, y2) = calcular_posiciones(theta1, theta2)
    line.set_xdata([0, x1, x2])
    line.set_ydata([0, y1, y2])
    fig.canvas.draw_idle()

# Vincular la actualización a los sliders
slider_theta1.on_changed(update)
slider_theta2.on_changed(update)

plt.grid(True)
plt.show()
