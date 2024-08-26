import pybullet as p
import pybullet_data
import time
import sympy as sp
# Iniciar el motor de física de PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Path para los URDF

# Cargar un entorno de planeo por defecto
planeId = p.loadURDF("plane.urdf")

# Definir las posiciones de las articulaciones iniciales
joint_positions = [200, 0, 0]

# Crear un robot simple con 3 articulaciones rotacionales
robot_id = p.loadURDF("r2d2.urdf", useFixedBase=True)

# Deshabilitar la gravedad para un control más fácil
p.setGravity(0, 0, 0)

# Configurar las articulaciones
num_joints = p.getNumJoints(robot_id)
for i in range(num_joints):
    p.resetJointState(robot_id, i, joint_positions[i % 3])

# Ejecutar la simulación en tiempo real
# Ejecutar la simulación
for t in range(100000):  # Un bucle para correr la simulación
    # Actualizar las posiciones de las articulaciones en tiempo real
    for i in range(p.getNumJoints(robot_id)):
        time.sleep(10)
        target_position = 0.5 * sp.sin(0.01 * t + i)  # Oscilar entre -0.5 y 0.5 radianes
        p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, targetPosition=target_position)
    
    # Avanzar la simulación
    p.stepSimulation()
    
    # Pausar por un pequeño tiempo para que la simulación sea visible
    time.sleep(1000000)

# Desconectar al finalizar
p.disconnect()

# Desconectar al finalizar
p.disconnect()
