import numpy as np
import matplotlib.pyplot as plt

# Parámetros del robot
l1 = 1  # Longitud del primer eslabón
l2 = 1  # Longitud del segundo eslabón

# Ángulos de las articulaciones (en radianes)
theta1 = np.pi /1
theta2 = np.pi / 4

# Posición de las articulaciones
x1 = l1 * np.cos(theta1)
y1 = l1 * np.sin(theta1)
x2 = x1 + l2 * np.cos(theta1 + theta2)
y2 = y1 + l2 * np.sin(theta1 + theta2)

# Graficar el robot
plt.figure()
plt.plot([0, x1, x2], [0, y1, y2], 'ro-')
plt.xlim(-2, 2)
plt.ylim(-2, 2)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Simulación de Robot 2 GDL')
plt.grid(True)
plt.show()
