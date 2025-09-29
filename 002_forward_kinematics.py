import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

planeId = p.loadURDF("plane.urdf")

# load your robot
pandaID = p.loadURDF("assets/franka_panda/panda.urdf", [0,0,0], useFixedBase=True)
n_joints = p.getNumJoints(pandaID)

link_names = []
for j in range(n_joints):
    info = p.getJointInfo(pandaID, j)
    joint_type = info[2]
    link_names.append(info[12].decode("utf-8"))
    if joint_type in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
        p.setJointMotorControl2(
            pandaID, j, p.VELOCITY_CONTROL, force=0.0,
        )

# load a frame marker
marker = p.loadURDF("assets/frame_marker/frame_marker.urdf", [0,0,1], useFixedBase=True, globalScaling=0.15)
link_name_to_visualize = "panda_hand"  # change this to visualize a different link
link_id_to_visualize = link_names.index(link_name_to_visualize)  # change this to visualize a different link

# run loop
for i in range(100000):

    # attach the marker
    link_state = p.getLinkState(pandaID, link_id_to_visualize)
    pos, orn = link_state[4], link_state[5]
    p.resetBasePositionAndOrientation(marker, pos, orn)

    p.stepSimulation()
    time.sleep(1./240.)