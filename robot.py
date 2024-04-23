from sensor import SENSOR
from motor import MOTOR
import constants as c
import numpy as np
import pybullet as p
import pyrosim.pyrosim as pyrosim
from pyrosim.neuralNetwork import NEURAL_NETWORK
import os


class ROBOT:
    def __init__(self, solutionID):
        self.Id = p.loadURDF("body.urdf")
        self.solutionID = solutionID

        pyrosim.Prepare_To_Simulate(self.Id)
        self.Prepare_To_Sense()
        self.Prepare_To_Act()
        self.nn = NEURAL_NETWORK(f"brains/brain{solutionID}.nndf")
        # os.system(f"rm brain{solutionID}.nndf")

    def Prepare_To_Sense(self):
        self.sensors = {}
        self.touch_matrix = np.zeros((c.steps, 4))
        for linkName in pyrosim.linkNamesToIndices:
            if (linkName == 'BackLowerLeg' or linkName == 'FrontLowerLeg' or linkName == 'LeftLowerLeg' or linkName == 'RightLowerLeg'):
                self.sensors[linkName] = SENSOR(linkName)

    def Sense(self, t):
        for i, sensor in enumerate(self.sensors.values()):
            # Store touch sensor value in binary matrix
            touch_val = sensor.Get_Value(t)
            print(touch_val)
            self.touch_matrix[t, i] = 1 if touch_val > 0 else -1

    def Prepare_To_Act(self):
        self.motors = {}

        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName)

    def Act(self):
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = \
                    self.nn.Get_Value_Of(neuronName) * c.motorJointRange
                self.motors[jointName].Set_Value(self, desiredAngle)

    def Think(self):
        self.nn.Update()

    def Get_Fitness(self):
        max_consecutive_airborne_steps = 0
        current_consecutive_airborne_steps = 0
        for t in range(c.steps):
            # Check if all legs are not touching the ground at timestep t
            print(self.touch_matrix[t])
            if np.all(self.touch_matrix[t] == -1):
                current_consecutive_airborne_steps += 1
            else:
                # If any leg touches the ground, reset the count
                max_consecutive_airborne_steps = max(max_consecutive_airborne_steps, current_consecutive_airborne_steps)
                current_consecutive_airborne_steps = 0

        # Check if the last segment of airborne steps is the longest
        max_consecutive_airborne_steps = max(max_consecutive_airborne_steps, current_consecutive_airborne_steps)

        f = open(f"fitness/tmp{self.solutionID}.txt", "w")
        f.write(f"{max_consecutive_airborne_steps}")
        f.close()

        os.system(f"mv fitness/tmp{self.solutionID}.txt fitness/fitness{self.solutionID}.txt")
