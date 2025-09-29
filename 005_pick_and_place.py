import pybullet as p
import pybullet_data
import numpy as np
import time
import math
from typing import List, Tuple

# ---------------------------- Helpers ----------------------------
def get_joint_indices_by_type(body_id: int,
                              include_types=(p.JOINT_REVOLUTE, p.JOINT_PRISMATIC)) -> Tuple[List[int], List[str]]:
    n = p.getNumJoints(body_id)
    idxs, names = [], []
    for j in range(n):
        info = p.getJointInfo(body_id, j)
        jtype = info[2]
        if jtype in include_types:
            idxs.append(j)
            names.append(info[1].decode("utf-8"))
    return idxs, names

def get_joint_index_by_name(body_id: int, name: str) -> int:
    n = p.getNumJoints(body_id)
    for j in range(n):
        info = p.getJointInfo(body_id, j)
        if info[1].decode("utf-8") == name:
            return j
    raise ValueError(f"Joint '{name}' not found.")

def read_limits(body_id: int, joint_indices: List[int]):
    lowers, uppers, ranges = [], [], []
    for j in joint_indices:
        info = p.getJointInfo(body_id, j)
        lo, hi = info[8], info[9]
        lowers.append(lo)
        uppers.append(hi)
        ranges.append(hi - lo)
    return lowers, uppers, ranges

def set_target_joint_values(body_id, joint_indices, rest_values,
                   pos_gain=0.3, vel_gain=1.0, torque=200):
    """Send PD control commands to hold given rest joint values."""
    for idx, j in enumerate(joint_indices):
        p.setJointMotorControl2(
            body_id,
            j,
            controlMode=p.POSITION_CONTROL,
            targetPosition=rest_values[idx],
            positionGain=pos_gain,
            velocityGain=vel_gain,
            force=torque
        )

def open_gripper(body_id: int, j1: int, j2: int, width_each=0.04, force=50):
    # Panda fingers are prismatic; joint2 may mimic joint1 in URDF, but command both for robustness.
    p.setJointMotorControl2(body_id, j1, p.POSITION_CONTROL, targetPosition=width_each, positionGain=0.01, velocityGain=0.1, force=force)
    p.setJointMotorControl2(body_id, j2, p.POSITION_CONTROL, targetPosition=width_each, positionGain=0.01, velocityGain=0.1, force=force)

def close_gripper(body_id: int, j1: int, j2: int, force=50):
    p.setJointMotorControl2(body_id, j1, p.POSITION_CONTROL, targetPosition=0.0, positionGain=0.01, velocityGain=0.1, force=force)
    p.setJointMotorControl2(body_id, j2, p.POSITION_CONTROL, targetPosition=0.0, positionGain=0.01, velocityGain=0.1, force=force)

def wait_steps(steps=240, eef_marker=None, panda=None, eef=None):
    for _ in range(steps):
        if eef_marker:
            pos, orn = p.getLinkState(panda, eef)[4:6]
            p.resetBasePositionAndOrientation(eef_marker, pos, orn)
        p.stepSimulation()
        time.sleep(1./240.)

def object_has_settled(body_id: int, lin_thresh=0.03, ang_thresh=0.1) -> bool:
    lin, ang = p.getBaseVelocity(body_id)
    v = np.linalg.norm(lin)
    w = np.linalg.norm(ang)
    return (v < lin_thresh) and (w < ang_thresh)

def smooth_joint_targets(body_id: int, joint_indices: List[int], ik_targets: List[float], step_size=0.02):
    out = []
    for k, j in enumerate(joint_indices):
        curr = p.getJointState(body_id, j)[0]
        tgt  = float(ik_targets[k])
        delta = tgt - curr
        if delta > step_size: new = curr + step_size
        elif delta < -step_size: new = curr - step_size
        else: new = tgt
        out.append(new)
    return out

def move_arm_to_pose(body_id: int,
                     arm_joint_indices: List[int],
                     eef_link_index: int,
                     target_pos: List[float],
                     target_orn: Tuple[float, float, float, float],
                     lowers: List[float],
                     uppers: List[float],
                     ranges: List[float],
                     max_secs=2.0,
                     pos_gain=0.2,
                     vel_gain=1.0,
                     torque=200,
                     dt=1./240.,
                     eef_marker=None):
    """IK + PD stepper until EE is near target or timeout."""
    max_steps = int(max_secs / dt)
    for _ in range(max_steps):
        rest = [p.getJointState(body_id, j)[0] for j in arm_joint_indices]
        ik = p.calculateInverseKinematics(
            body_id, eef_link_index,
            target_pos, target_orn,
            lowerLimits=lowers, upperLimits=uppers, jointRanges=ranges, restPoses=rest
        )
        ik = list(ik[:len(arm_joint_indices)])  # trim in case IK returns extra joints
        smoothed = smooth_joint_targets(body_id, arm_joint_indices, ik, step_size=0.008)
        for idx, j in enumerate(arm_joint_indices[:-2]):
            p.setJointMotorControl2(body_id, j, p.POSITION_CONTROL,
                                    targetPosition=smoothed[idx],
                                    positionGain=pos_gain, velocityGain=vel_gain, force=torque)
            
        if eef_marker:
            pos, orn = p.getLinkState(body_id, eef_link_index)[4:6]
            p.resetBasePositionAndOrientation(eef_marker, pos, orn)

        p.stepSimulation()
        time.sleep(dt)

        # quick EE error check
        pos, orn = p.getLinkState(body_id, eef_link_index)[4:6]
        pos_err = np.linalg.norm(np.array(pos) - np.array(target_pos))
        orn_err = 1 - abs(np.dot(orn, target_orn))  # quaternion difference proxy
        if pos_err < 0.001 and orn_err < 0.0001:
            return True
    return False

def spawn_keypose_markers(frame_urdf, keyposes, orn, scale=0.1):
    """Spawn frame markers + text labels for each keypose."""
    for name, pos in keyposes.items():
        p.loadURDF(frame_urdf, pos, orn, useFixedBase=True, globalScaling=scale)
        # Add text above the frame
        text_pos = [pos[0], pos[1], pos[2] + 0.01]
        p.addUserDebugText(name, text_pos, textColorRGB=[1, 1, 0], textSize=1.2)

# ---------------------------- Main ----------------------------

def main():
    # --- PyBullet setup ---
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    plane = p.loadURDF("plane.urdf")

    # --- Load Panda ---
    panda = p.loadURDF("assets/franka_panda/panda.urdf", [0, 0, 0], useFixedBase=True)
    n = p.getNumJoints(panda)

    # Arm joints + names
    arm_joints, arm_joint_names = get_joint_indices_by_type(panda)
    # Disable default motors first
    for j in arm_joints:
        p.setJointMotorControl2(panda, j, p.VELOCITY_CONTROL, force=0.0)

    # EE link index (by link name)
    link_names = [p.getJointInfo(panda, j)[12].decode("utf-8") for j in range(n)]
    eef_link_name = "panda_hand"
    eef = link_names.index(eef_link_name)

    # Gripper joint indices (robust to URDF variants)
    j_f1 = get_joint_index_by_name(panda, "panda_finger_joint1")
    j_f2 = get_joint_index_by_name(panda, "panda_finger_joint2")

    # Limits
    lowers, uppers, ranges = read_limits(panda, arm_joints)

    # Neutral pose
    neutral = []
    for j, lo, ra in zip(arm_joints, lowers, ranges):
        neutral.append(lo + 0.5 * ra)
    # A nicer elbow-up posture tweak (optional)
    if len(neutral) >= 7:
        neutral[-3] = 0.74
    for idx, j in enumerate(arm_joints):
        p.resetJointState(panda, j, neutral[idx])

    # --- Load visual frame markers (optional, nice to see) ---
    frame_urdf = "assets/frame_marker/frame_marker_target.urdf"
    current_frame_urdf = "assets/frame_marker/frame_marker.urdf"
    eef_marker = p.loadURDF(current_frame_urdf, [0, 0, 0], useFixedBase=True, globalScaling=0.18)

    # --- Load cubes ---
    # Goal (green, visual only, randomized)
    rng = np.random.default_rng()
    goal_x = float(rng.uniform(0.4, 0.7))
    goal_y = float(rng.uniform(0.1, 0.4))
    goal_z = 0.025
    goal_pos = [goal_x, goal_y, goal_z]
    goal_orn = p.getQuaternionFromEuler([0, 0, 0])
    green = p.loadURDF("assets/cubes/5cm_green_cube.urdf", goal_pos, goal_orn, useFixedBase=True)

    # Object (red, dynamic), random drop pose
    obj_x = float(rng.uniform(0.4, 0.7))
    obj_y = float(rng.uniform(-0.4, -0.1))
    obj_z = 0.40
    obj_pos0 = [obj_x, obj_y, obj_z]
    obj_orn0 = p.getQuaternionFromEuler([0, 0, 0])
    red = p.loadURDF("assets/cubes/5cm_red_cube.urdf", obj_pos0, obj_orn0, useFixedBase=False)

    # --- Let the cube fall & settle ---
    set_target_joint_values(panda, arm_joints, neutral)
    wait_steps(240, eef_marker, panda, eef)  # 1 sec wait
    while not object_has_settled(red):
        wait_steps(1, eef_marker, panda, eef)

    # Read settled object pose
    obj_pos, obj_orn = p.getBasePositionAndOrientation(red)
    obj_pos = list(obj_pos)

    # --- Grasping strategy ---
    down_orn = p.getQuaternionFromEuler([math.pi, 0, 0])
    cube_half = 0.025
    approach_height = 0.20
    grasp_clearance = 0.10
    lift_height = 0.40

    pregrasp = [obj_pos[0], obj_pos[1], obj_pos[2] + approach_height]
    grasp    = [obj_pos[0], obj_pos[1], cube_half + grasp_clearance]
    lift     = [grasp[0], grasp[1], lift_height]
    place_above = [goal_pos[0], goal_pos[1], lift_height]
    preplace    = [goal_pos[0], goal_pos[1], cube_half + approach_height]
    place    = [goal_pos[0], goal_pos[1], cube_half + grasp_clearance + 0.01]

    # --- Show static markers for key target poses ---
    keyposes = {
        "pregrasp": pregrasp,
        "grasp": grasp,
        "lift": lift,
        "place_above": place_above,
        "preplace": preplace,
        "place": place,
    }
    spawn_keypose_markers(frame_urdf, keyposes, down_orn)
    for name, pos in keyposes.items():
        p.loadURDF(frame_urdf, pos, down_orn, useFixedBase=True, globalScaling=0.1)

    # --- Execution loop ---


    # Open gripper for approach
    open_gripper(panda, j_f1, j_f2, width_each=0.04, force=200)
    wait_steps(20, eef_marker, panda, eef)

    # Move to pregrasp
    move_arm_to_pose(panda, arm_joints, eef, pregrasp, down_orn, lowers, uppers, ranges, max_secs=3.5, eef_marker=eef_marker)

    # Move to grasp
    move_arm_to_pose(panda, arm_joints, eef, grasp, down_orn, lowers, uppers, ranges, max_secs=3.0, eef_marker=eef_marker)

    # Close gripper
    close_gripper(panda, j_f1, j_f2, force=200)
    # give time to squeeze
    wait_steps(60, eef_marker, panda, eef)

    # Lift straight up
    move_arm_to_pose(panda, arm_joints, eef, lift, down_orn, lowers, uppers, ranges, max_secs=3.0, eef_marker=eef_marker)

    # Move above the goal
    move_arm_to_pose(panda, arm_joints, eef, place_above, down_orn, lowers, uppers, ranges, max_secs=3.5, eef_marker=eef_marker)

    # Move to preplace
    move_arm_to_pose(panda, arm_joints, eef, preplace, down_orn, lowers, uppers, ranges, max_secs=3.0, eef_marker=eef_marker)

    # Lower to place
    move_arm_to_pose(panda, arm_joints, eef, place, down_orn, lowers, uppers, ranges, max_secs=3.0, eef_marker=eef_marker)

    # Open gripper to release
    open_gripper(panda, j_f1, j_f2, width_each=0.04, force=80)
    wait_steps(60, eef_marker, panda, eef)

    # Retreat up
    move_arm_to_pose(panda, arm_joints, eef, place_above, down_orn, lowers, uppers, ranges, max_secs=3.0, eef_marker=eef_marker)

    # Idle loop + update EEF marker (optional)
    wait_steps(1000000, eef_marker, panda, eef)

if __name__ == "__main__":
    main()
