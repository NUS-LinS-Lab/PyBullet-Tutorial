# PyBullet Tutorial (TA Session)

This is a PyBullet tutorial for CS4278/CS5478 Intelligent Robots: Algorithms and Systems. 
This repository contains a step-by-step introduction to **PyBullet** for robotics simulation.
The tutorial is structured into **six example scripts**.
It covers basic simulation, loading robots, forward & inverse kinematics, control, and a full pick-and-place demo with the Franka Panda arm.

---

## 📂 Files Overview

### 0. Hello PyBullet — `000_hello_pybullet.py`

* Minimal example: connect to PyBullet, load a plane and a falling cube, step the simulation.
* Teaches how to:

  * Start/stop simulation
  * Load URDF assets
  * Get object states (`getBasePositionAndOrientation`)

---

### 1. Load a Robot — `001_get_a_robot.py`

* Load the **Franka Panda** arm and keep it fixed at the base.
* Demonstrates:

  * Inspecting joints
  * Disabling default motors
  * Understanding fixed base vs free base

---

### 2. Forward Kinematics (FK) — `002_forward_kinematics.py`

* Visualize robot link frames using a small URDF marker.
* Demonstrates:

  * `getLinkState` to get a link’s world transform
  * Attaching markers to links for FK visualization
  * Understanding Panda’s link hierarchy

---

### 3. Inverse Kinematics (IK) — `003_inverse_kinematics.py`

* Add GUI sliders for an interactive IK demo.
* Demonstrates:

  * `calculateInverseKinematics`
  * Using GUI sliders to set EE target pose
  * Comparing target vs current EE frame

---

### 4. PD Control — `004_pd_control.py`

* Replace direct joint resets with **PD controllers**.
* Demonstrates:

  * `POSITION_CONTROL` mode
  * Effects of P/D gains and torque limits

---

### 5. Pick and Place — `005_pick_and_place.py`

* Full pipeline: detect cube, compute grasp poses, move Panda with IK+PD, control gripper, place cube at a target.
* Features:

  * Random red cube dropped in front of robot
  * Random green cube as goal (visual only)
  * Keyposes (pregrasp, grasp, lift, place) visualized with markers + labels
  * Gripper open/close with prismatic joints
  * Helpers for joint limits, object settling, and smooth IK

---

## 🛠️ Setup

1. Install dependencies:

   ```bash
   pip install pybullet numpy
   ```

2. Clone this repo and run examples:

   ```bash
   python 000_hello_pybullet.py
   python 001_get_a_robot.py
   ...
   python 005_pick_and_place.py
   ```

3. Assets:

   * Place additional URDFs under `assets/`:

     * `assets/plane/plane.urdf`
     * `assets/franka_panda/panda.urdf`
     * `assets/frame_marker/`
     * `assets/cubes/5cm_green_cube.urdf`, `5cm_red_cube.urdf`

---


## References
- [PyBullet Quickstart Guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit?tab=t.0#heading=h.2ye70wns7io3).
- [PyBullet Examples](https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet/examples). 
