import pybullet as p
import time

# 'connecting' to the physics simulation
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setGravity(0,0,-10)

# load the scene 
planeId = p.loadURDF("assets/plane/plane.urdf")

startPos = [0,0,0]
startOrientation = p.getQuaternionFromEuler([0,0,0])
pandaID = p.loadURDF("assets/franka_panda/panda.urdf", startPos, startOrientation, 
                   useFixedBase=True)

n_joints = p.getNumJoints(pandaID)
for j in range(n_joints):
    info = p.getJointInfo(pandaID, j)
    joint_type = info[2]
    if joint_type in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
        p.setJointMotorControl2(
            pandaID, j, p.VELOCITY_CONTROL, force=0.0,
        )

# run the simulation
for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
    
p.disconnect()
