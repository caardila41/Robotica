# import cv2
# import mediapipe as mp
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

# # Inicializar MediaPipe para detección de manos
# mp_hands = mp.solutions.hands
# hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1)
# mp_draw = mp.solutions.drawing_utils

# # Captura de video desde la cámara
# cap = cv2.VideoCapture(0)

# while True:
#     success, img = cap.read()
#     if not success:
#         break

#     img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # Convertir a RGB para MediaPipe
#     results = hands.process(img_rgb)  # Procesar la imagen y detectar manos

#     if results.multi_hand_landmarks:  # Si se detectan manos
#         for hand_landmarks in results.multi_hand_landmarks:
#             # Crear listas para almacenar las coordenadas X, Y, Z
#             x_vals = []
#             y_vals = []
#             z_vals = []

#             for lm in hand_landmarks.landmark:
#                 h, w, c = img.shape
#                 x_vals.append(lm.x)
#                 y_vals.append(lm.y)
#                 z_vals.append(lm.z)

#             # Dibujar los puntos clave en el plano 3D
#             fig = plt.figure()
#             ax = fig.add_subplot(111, projection='3d')
#             ax.scatter(x_vals, y_vals, z_vals, c='r', marker='o')

#             ax.set_xlabel('X')
#             ax.set_ylabel('Y')
#             ax.set_zlabel('Z')
#             ax.set_title('3D Hand Landmarks')

#             plt.show()

#     cv2.imshow("Hand Tracking", img)  # Mostrar el video con el seguimiento
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# cap.release()
# cv2.destroyAllWindows()


import cv2
import mediapipe as mp
import plotly.graph_objs as go
from plotly.subplots import make_subplots

# Inicializar MediaPipe para detección de manos
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1)  # Detectar solo una mano para simplificar
mp_draw = mp.solutions.drawing_utils

# Captura de video desde la cámara
cap = cv2.VideoCapture(0)

# Configuración inicial de la visualización 3D
fig = make_subplots(rows=1, cols=1, specs=[[{'type': 'scatter3d'}]])
scatter = go.Scatter3d(x=[], y=[], z=[], mode='markers', marker=dict(size=5, color='red'))
fig.add_trace(scatter)

# Mostrar el gráfico vacío
fig.show()

while True:
    success, img = cap.read()
    if not success:
        break

    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # Convertir a RGB para MediaPipe
    results = hands.process(img_rgb)  # Procesar la imagen y detectar manos

    if results.multi_hand_landmarks:  # Si se detectan manos
        for hand_landmarks in results.multi_hand_landmarks:
            # Crear listas para almacenar las coordenadas X, Y, Z
            x_vals = []
            y_vals = []
            z_vals = []

            for lm in hand_landmarks.landmark:
                h, w, c = img.shape
                x_vals.append(lm.x)
                y_vals.append(lm.y)
                z_vals.append(lm.z)

            # Actualizar los datos del gráfico 3D
            scatter.update({'x': x_vals, 'y': y_vals, 'z': z_vals})

            # Renderizar el gráfico actualizado
            fig.update_traces(scatter)
            fig.update_layout(scene=dict(aspectmode='cube'))  # Mantener aspecto cúbico
            fig.show()

    cv2.imshow("Hand Tracking", img)  # Mostrar el video con el seguimiento
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
