import cv2
import mediapipe as mp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import threading

# Variables globales para compartir datos entre hilos
x_vals, y_vals, z_vals = [], [], []
lock = threading.Lock()  # Un candado para asegurar que los datos se manipulan de forma segura

# Función para capturar video y procesar puntos clave
def captura_y_procesa():
    global x_vals, y_vals, z_vals
    mp_hands = mp.solutions.hands
    
   
    hands = mp_hands.Hands()
    mp_draw = mp.solutions.drawing_utils

    cap = cv2.VideoCapture(0)

    while True:
        success, img = cap.read()
        if not success:
            break

        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = hands.process(img_rgb)
   

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_draw.draw_landmarks(img, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                h, w, c = img.shape
                escala_z = 1000

                with lock:  # Usar un candado para asegurar el acceso seguro a las listas
                    x_vals = []
                    y_vals = []
                    z_vals = []

                    for lm in hand_landmarks.landmark:
                        cx = int(lm.x * w)
                        cy = int(lm.y * h)
                        cz = int(lm.z * escala_z)
                        x_vals.append(cx)
                        y_vals.append(cy)
                        z_vals.append(cz)

        cv2.imshow("Hand Tracking", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Función para visualizar los puntos 3D
def visualiza_3d():
    global x_vals, y_vals, z_vals

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    while True:
        with lock:  # Usar un candado para asegurar el acceso seguro a las listas
            ax.cla()  # Limpiar el gráfico antes de volver a dibujarlo
            ax.scatter(x_vals, y_vals, z_vals, c='r', marker='o')

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Hand Landmarks')

        plt.pause(0.1)  # Pausa pequeña para actualizar la visualización

# Crear y empezar los hilos
hilo_captura = threading.Thread(target=captura_y_procesa)
hilo_visualiza = threading.Thread(target=visualiza_3d)

hilo_captura.start()
hilo_visualiza.start()

hilo_captura.join()
hilo_visualiza.join()
