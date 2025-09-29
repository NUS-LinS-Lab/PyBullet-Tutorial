
import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setGravity(0,0,-10)
planeId = p.loadURDF("assets/plane/plane.urdf")
startPos = [0,0,4]
startOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("assets/box/box.urdf",startPos, startOrientation)
for i in range (10000):
    p.stepSimulation()
    cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
    print(cubePos,cubeOrn)
    time.sleep(1./240.)
p.disconnect()
