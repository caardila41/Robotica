import pybullet as p
import pybullet_data

# Conectar al simulador
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Cargar un robot URDF (ejemplo: KUKA iiwa)
robot_id = p.loadURDF("kuka_iiwa/model.urdf")

# Obtener la cinemática directa
joint_positions = [0, 0, 0, 0, 0, 0, 0]  # Ajustar los valores para los 7 DOF del KUKA iiwa
end_effector_state = p.getLinkState(robot_id, 6)  # Link 6 es el efector final
print("End Effector Position:", end_effector_state[4])

# Obtener la cinemática inversa
target_position = [0.4, 0.2, 0.3]
ik_solution = p.calculateInverseKinematics(robot_id, 6, target_position)
print("IK Solution:", ik_solution)
