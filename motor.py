import constants as c

import pybullet as p
import pyrosim.pyrosim as pyrosim


class MOTOR:
    def __init__(self, jointName):
        self.jointName = jointName

    def Set_Value(self, robot, desiredAngle):
        pyrosim.Set_Motor_For_Joint(
                bodyIndex=robot.Id,
                jointName=self.jointName,
                controlMode=p.POSITION_CONTROL,
                targetPosition=desiredAngle,
                maxForce=c.maxForce)
