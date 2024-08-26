import pybullet as p
import pybullet_data
import time

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
    # Aquí, como ejemplo, actualizamos los ángulos a una posición específica
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

# Desconectar la simulación (si no se usa un bucle infinito)
# p.disconnect()
