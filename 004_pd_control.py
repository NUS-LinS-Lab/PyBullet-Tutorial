import pybullet as p
import pybullet_data
import numpy as np
import time
import math

# ---------------- Setup ----------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

planeId = p.loadURDF("plane.urdf")

# Load Panda
pandaID = p.loadURDF("assets/franka_panda/panda.urdf", [0,0,0], useFixedBase=True)
n_joints = p.getNumJoints(pandaID)

link_names = []
controlled_joints = []
lowers, uppers, ranges, rest = [], [], [], []
for j in range(n_joints):
    info = p.getJointInfo(pandaID, j)
    joint_type = info[2]
    link_names.append(info[12].decode("utf-8"))
    if joint_type in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
        controlled_joints.append(j)
        lowers.append(info[8])
        uppers.append(info[9])
        ranges.append(info[9] - info[8])
        rest.append(p.getJointState(pandaID, j)[0])
        # disable default motor
        p.setJointMotorControl2(pandaID, j, p.POSITION_CONTROL, targetPosition=0, force=0.0)

neutral_joint_values = np.array(lowers) + 0.5 * np.array(ranges)
neutral_joint_values[-3] = 0.74
for idx, j in enumerate(controlled_joints):
    p.resetJointState(pandaID, j, neutral_joint_values[idx])
    
# Get the link id for the hand (EEF)
eef_link_name = "panda_hand"
eef_id = link_names.index(eef_link_name)

# Load frame markers
target_marker = p.loadURDF("assets/frame_marker/frame_marker.urdf", [0.6,0,0.6],
                           useFixedBase=True, globalScaling=0.15)
eef_marker = p.loadURDF("assets/frame_marker/frame_marker_target.urdf", [0,0,0],
                        useFixedBase=True, globalScaling=0.25)

# ---------------- GUI Sliders ----------------
slider_x = p.addUserDebugParameter("target_x", 0.2, 0.8, 0.6)
slider_y = p.addUserDebugParameter("target_y", -0.5, 0.5, 0.0)
slider_z = p.addUserDebugParameter("target_z", 0.1, 0.8, 0.6)

slider_roll  = p.addUserDebugParameter("target_roll", -math.pi, math.pi, 3.14)
slider_pitch = p.addUserDebugParameter("target_pitch", -math.pi, math.pi, 0)
slider_yaw   = p.addUserDebugParameter("target_yaw", -math.pi, math.pi, 0)

# ---------------- Run Loop ----------------
for i in range(100000):

    # Read target pose from sliders
    target_pos = [
        p.readUserDebugParameter(slider_x),
        p.readUserDebugParameter(slider_y),
        p.readUserDebugParameter(slider_z),
    ]
    r = p.readUserDebugParameter(slider_roll)
    pch = p.readUserDebugParameter(slider_pitch)
    y = p.readUserDebugParameter(slider_yaw)
    target_orn = p.getQuaternionFromEuler([r, pch, y])

    # Place target marker
    p.resetBasePositionAndOrientation(target_marker, target_pos, target_orn)

    # Inverse Kinematics
    current_joint_values = [p.getJointState(pandaID, j)[0] for j in controlled_joints]
    joint_positions = p.calculateInverseKinematics(
        pandaID,
        eef_id,
        target_pos,
        target_orn,
    )

    # Apply PD control instead of reset
    for idx, joint_idx in enumerate(controlled_joints):
        p.setJointMotorControl2(
            pandaID,
            joint_idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=joint_positions[idx],
            positionGain=0.2,   # P gain (default 0.1)
            velocityGain=1.0,   # D gain (default 1.0)
            force=200           # max torque
        )

    # Visualize current EEF frame
    eef_state = p.getLinkState(pandaID, eef_id)
    curr_pos, curr_orn = eef_state[4], eef_state[5]
    p.resetBasePositionAndOrientation(eef_marker, curr_pos, curr_orn)

    p.stepSimulation()
    time.sleep(1./240.)
