import cv2
import mediapipe as mp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import threading

import pybullet as p
import pybullet_data
import time

import math

# Variables globales para compartir datos entre hilos
x_vals, y_vals, z_vals = [], [], []
angles_x, angles_y, angles_z = [], [], []
lock = threading.Lock()  # Un candado para asegurar que los datos se manipulan de forma segura

# Función para capturar video y procesar puntos clave
def captura_y_procesa():
    global x_vals, y_vals, z_vals, angles_x, angles_y, angles_z
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
                    angles_x = []
                    angles_y = []
                    angles_z = []

                    for lm in hand_landmarks.landmark:
                        cx = int(lm.x * w)
                        cy = int(lm.y * h)
                        cz = int(lm.z * escala_z)
                        x_vals.append(cx)
                        y_vals.append(cy)
                        z_vals.append(cz)

                        # Calcular los ángulos con respecto a cada eje
                        theta_x = math.degrees(math.atan2(math.sqrt(cy**2 + cz**2), cx))
                        theta_y = math.degrees(math.atan2(math.sqrt(cx**2 + cz**2), cy))
                        theta_z = math.degrees(math.atan2(math.sqrt(cx**2 + cy**2), cz))
                        
                        angles_x.append(theta_x)
                        angles_y.append(theta_y)
                        angles_z.append(theta_z)

        cv2.imshow("Hand Tracking", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Función para visualizar los puntos 3D
def visualiza_3d():
    global x_vals, y_vals, z_vals, angles_x, angles_y, angles_z

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    while True:
        with lock:  # Usar un candado para asegurar el acceso seguro a las listas
            ax.cla()  # Limpiar el gráfico antes de volver a dibujarlo
            ax.scatter(x_vals, y_vals, z_vals, c='r', marker='o')
            for i in range(len(x_vals)):
                ax.text(x_vals[i], y_vals[i], z_vals[i],
                        f'({angles_x[i]:.1f}, {angles_y[i]:.1f}, {angles_z[i]:.1f})', 
                        color='blue')

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Hand Landmarks with Angles')

        plt.pause(0.1)  # Pausa pequeña para actualizar la visualización

def run_simulation():
    # Conectar al simulador
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Cargar un robot URDF (ejemplo: KUKA iiwa)
    robot_id = p.loadURDF("kuka_iiwa/model.urdf")

    # Configurar la simulación de tiempo real
    p.setRealTimeSimulation(0)

    # Bucle para controlar las articulaciones
    while True:
        # Actualizar los ángulos de las articulaciones
        target_positions = [1.3, 1.3, 0.3, 0.4, 0.5, 0.6, 0.7]
        
        for joint_index in range(p.getNumJoints(robot_id)):
            p.setJointMotorControl2(bodyUniqueId=robot_id,
                                    jointIndex=joint_index,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=target_positions[joint_index])

        # Avanzar la simulación
        p.stepSimulation()
        
        # Pausa para permitir que la simulación se vea de forma más natural
        time.sleep(0.01)  # 10 ms para controlar la velocidad de la simulación

    # Desconectar la simulación (nunca se llega a esta línea en este bucle infinito)
    # p.disconnect()

# Crear un hilo para ejecutar la simulación
simulation_thread = threading.Thread(target=run_simulation)



# Crear y empezar los hilos
hilo_visualiza = threading.Thread(target=visualiza_3d)
hilo_captura = threading.Thread(target=captura_y_procesa)

# Iniciar el hilo
simulation_thread.start()
hilo_captura.start()
hilo_visualiza.start()
simulation_thread.join()
hilo_captura.join()
hilo_visualiza.join()
