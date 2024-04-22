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
        self.touch_matrix = np.zeros((c.steps, len(pyrosim.linkNamesToIndices)))
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    def Sense(self, t):
        for i, sensor in enumerate(self.sensors.values()):
            # Store touch sensor value in binary matrix
            touch_val = sensor.Get_Value(t)
            print(touch_val)
            self.touch_matrix[t, i] = 1 if touch_val > 0 else -1
        #print(self.touch_matrix)

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
        basePositionAndOrientation = p.getBasePositionAndOrientation(self.Id)
        basePosition = basePositionAndOrientation[0]
        zPosition = basePosition[2]
        #print(self.touch_matrix)
        
        num_minus_ones = np.count_nonzero(self.touch_matrix == -1)
        f = open(f"fitness/tmp{self.solutionID}.txt", "w")
        f.write(f"{num_minus_ones}")
        f.close()

        os.system(f"mv fitness/tmp{self.solutionID}.txt fitness/fitness{self.solutionID}.txt")
