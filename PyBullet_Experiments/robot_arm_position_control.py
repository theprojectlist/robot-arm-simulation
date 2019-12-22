
import pybullet as p
import time
import pybullet_data
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally

p.loadURDF("plane.urdf")
kuka = p.loadURDF("kuka_iiwa/model.urdf")

p.setGravity(0,0,-10)
# p.addUserDebugText("tip", [0,0,0.05],textColorRGB=[1,0,0],textSize=1.5,trackObjectUniqueId=kuka, trackLinkIndex=6)
# p.addUserDebugLine([0,0,0],[0.1,0,0],[1,0,0],kuka,6)
# p.addUserDebugLine([0,0,0],[0,0.1,0],[0,1,0],kuka,6)
# p.addUserDebugLine([0,0,0],[0,0,0.1],[0,0,1],kuka,6)
# p.addUserDebugParameter()
p.setRealTimeSimulation(0)


target_x_id = p.addUserDebugParameter("x", -1, 1, 0)
target_y_id= p.addUserDebugParameter("y", -1, 1, 0)
target_z_id = p.addUserDebugParameter("z", -1, 1, 0)


# print(p.getJointInfo(kuka,6))
targetVel = 1
maxForce = 500
while (True):

    target_x = p.readUserDebugParameter(target_x_id)
    target_y = p.readUserDebugParameter(target_y_id)
    target_z = p.readUserDebugParameter(target_z_id)
    p.stepSimulation()
    positions = p.calculateInverseKinematics(kuka, 6, [target_x,target_y,target_z])
    p.setJointMotorControlArray(kuka, [0,1,2,3,4,5,6], p.POSITION_CONTROL,
        targetPositions = positions,
        forces = [maxForce, maxForce,maxForce, maxForce,maxForce, maxForce, maxForce])

    time.sleep(1./240.)