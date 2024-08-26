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
vector_x, vector_y, vector_z = [], [], []
angles_x, angles_y, angles_z = [], [], []
lock = threading.Lock()  # Un candado para asegurar que los datos se manipulan de forma segura

# Función para capturar video y procesar puntos clave
def captura_y_procesa():
    global x_vals, y_vals, z_vals, vector_x, vector_y, vector_z, angles_x, angles_y, angles_z
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
                    vector_x = []
                    vector_y = []
                    vector_z = []
                    angles_x = []
                    angles_y = []
                    angles_z = []

                    # Acceder a los puntos de la mano
                    for lm in hand_landmarks.landmark:
                        x_vals.append(lm.x * w)
                        y_vals.append(lm.y * h)
                        z_vals.append(lm.z * escala_z)

                    # Elegir los puntos 0 y 9 (puedes cambiar el segundo punto)
                    punto_0 = hand_landmarks.landmark[0]  # Punto de referencia
                    punto_n = hand_landmarks.landmark[9]  # Punto destino

                    x0, y0, z0 = punto_0.x * w, punto_0.y * h, punto_0.z * escala_z
                    xn, yn, zn = punto_n.x * w, punto_n.y * h, punto_n.z * escala_z

                    vector_x = [x0, xn]
                    vector_y = [y0, yn]
                    vector_z = [z0, zn]

                    # # Calcular las diferencias entre las coordenadas
                    dx = xn - x0
                    dy = yn - y0
                    dz = zn - z0

                   
                    # Calcular los ángulos con respecto a cada eje
                    theta_x = math.degrees(math.atan2(math.sqrt(dy**2 + dz**2), dx))
                    theta_y = math.degrees(math.atan2(math.sqrt(dx**2 + dz**2), dy))
                    theta_z = math.degrees(math.atan2(math.sqrt(dx**2 + dy**2), dz))
                    
                    angles_x.append(theta_x)
                    angles_y.append(theta_y)
                    angles_z.append(theta_z)

        cv2.imshow("Hand Tracking", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Función para visualizar todos los puntos, el vector y los ángulos en 3D
def visualiza_3d():
    global x_vals, y_vals, z_vals, vector_x, vector_y, vector_z, angles_x, angles_y, angles_z

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    while True:
        with lock:  # Usar un candado para asegurar el acceso seguro a las listas
            if len(x_vals) > 0:
                ax.cla()  # Limpiar el gráfico antes de volver a dibujarlo
                
                # Dibujar todos los puntos de la mano
                ax.scatter(x_vals, y_vals, z_vals, c='r', marker='o')

                # Dibujar el vector entre el punto 0 y el punto n
                if len(vector_x) == 2:
                    ax.plot(vector_x, vector_y, vector_z, c='b', marker='o')
                    ax.text(vector_x[1], vector_y[1], vector_z[1],
                            f'({angles_x[0]:.1f}, {angles_y[0]:.1f}, {angles_z[0]:.1f})', 
                            color='blue')

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Hand Landmarks and Vector with Angles')

        plt.pause(0.1)  # Pausa pequeña para actualizar la visualización

def run_simulation():
    global angles_y, angles_x,angles_z,lock

    # Conectar al simulador
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Cargar un robot URDF (ejemplo: KUKA iiwa)
    robot_id = p.loadURDF("kuka_iiwa/model.urdf")

    # Configurar la simulación de tiempo real
    p.setRealTimeSimulation(0)
    angle_x =1.0
    angle_y = 1.0
    angle_z = 1.0
    # Bucle para controlar las articulaciones
    while True:
        with lock:  # Usar un candado para acceder a angles_x de forma segura
            if angles_z:
                angle_x = angles_x[0] * math.pi / 180  # Convertir a radianes
                angle_y = angles_y[0] * math.pi / 180  # Convertir a radianes
                angle_z = angles_z[0] * math.pi / 180  # Convertir a radianes
            

        # Actualizar los ángulos de las articulaciones
        target_positions = [angle_z, angle_x, angle_y, 0.5, 0.6, 0.7]
        
        for joint_index in range(p.getNumJoints(robot_id) -1):
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


# Crear y empezar los hilos
hilo_visualiza = threading.Thread(target=visualiza_3d)
hilo_captura = threading.Thread(target=captura_y_procesa)
# Crear un hilo para ejecutar la simulación
simulation_thread = threading.Thread(target=run_simulation)

# Iniciar el hilo

hilo_captura.start()
hilo_visualiza.start()
simulation_thread.start()
simulation_thread.join()
hilo_captura.join()
hilo_visualiza.join()
