import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
print(pybullet_data.getDataPath())
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("cube_rotate.urdf",cubeStartPos, cubeStartOrientation)
maxForce = 50
targetVel = .1
print(p.getJointInfo(boxId,0))
parameter = p.addUserDebugParameter("targetVel", -10, 10, 0)
for i in range (10000):
    targetVel = p.readUserDebugParameter(parameter)
    p.setJointMotorControl2(bodyUniqueId=boxId, 
    jointIndex=0, 
    controlMode=p.VELOCITY_CONTROL,
    targetVelocity = targetVel,
    force = maxForce)
    # p.setJointMotorControl2(bodyUniqueId=boxId, 
    # jointIndex=1, 
    # controlMode=p.VELOCITY_CONTROL,
    # targetVelocity = 0,
    # force=0)

    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
# print(cubePos,cubeOrn)
print(p.getJointInfo(boxId, 10))

p.disconnect()
