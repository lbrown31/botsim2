from world import WORLD
from robot import ROBOT
import constants as c

import pybullet as p
import pybullet_data
from time import sleep


class SIMULATION:
    def __init__(self, directOrGUI, simulationID):
        self.directOrGUI = directOrGUI

        if directOrGUI == "DIRECT":
            self.physicsClient = p.connect(p.DIRECT)
        elif directOrGUI == "GUI":
            self.physicsClient = p.connect(p.GUI)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)

        self.world = WORLD()
        self.robot = ROBOT(simulationID)

    def __del__(self):
        p.disconnect()

    def Run(self):
        for t in range(c.steps):
            p.stepSimulation()

            self.robot.Sense(t)
            self.robot.Think()
            self.robot.Act()

            if self.directOrGUI == "GUI":
                sleep(c.sleep)

    def Get_Fitness(self):
        self.robot.Get_Fitness()
