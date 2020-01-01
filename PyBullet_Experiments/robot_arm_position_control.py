
import pybullet as p
import time
import pybullet_data
from pyquaternion import Quaternion

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
        forces = [maxForce, maxForce, maxForce, maxForce, maxForce, maxForce, maxForce])


    ### Camera Rendering ###

    position, orientation,_,_,_,_ = p.getLinkState(kuka, 6)

    print(orientation)

    initialVector = [0,0,1]
    rotation = Quaternion(orientation[3],orientation[0],orientation[1],orientation[2])
    rotatedVector = rotation.rotate(initialVector)


    viewMatrix = p.computeViewMatrix(
    cameraEyePosition=position,
    cameraTargetPosition=[position[0]+rotatedVector[0],position[1]+rotatedVector[1],position[2]+rotatedVector[2]],
    cameraUpVector=[0, 0, 1])

    projectionMatrix = p.computeProjectionMatrixFOV(
            fov=45.0,
            aspect=1.0,
            nearVal=0.1,
            farVal=3.1)

    width, height, rgbImg, depthImg, segImg = p.getCameraImage(
            width=224, 
            height=224,
            viewMatrix=viewMatrix,
            projectionMatrix=projectionMatrix)

    p.addUserDebugLine([position[0],position[1],position[2]],[position[0]+rotatedVector[0],position[1]+rotatedVector[1],position[2]+rotatedVector[2]], lifeTime=1, lineColorRGB=[0,1,0])
    # p.addUserDebugLine([0,0,0], rotatedVector, lifeTime=1, lineColorRGB=[1,0,0])

    time.sleep(1./240.)