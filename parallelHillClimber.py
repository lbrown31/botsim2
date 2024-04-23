from solution import SOLUTION
import constants as c
import copy
import os
import pyrosim.pyrosim as pyrosim


class PARALLEL_HILL_CLIMBER:
    def __init__(self):
        self.parents = {}
        self.nextAvailableID = 0

        os.system("rm brains/brain*.nndf")
        os.system("rm fitness/fitness*.txt")
        os.system("rm fitness/tmp*.txt")


        self.Create_World()
        self.Create_Body()

        for i in range(c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1

    def Evolve(self):
        self.parents[0].Start_Simulation("GUI", background=False)
        self.Evaluate(self.parents)

        for currentGeneration in range(c.numberOfGenerations):
            print(f"generation: {currentGeneration}")
            self.Evolve_For_One_Generation()

    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.Evaluate(self.children)
        
        #self.Print()
        self.Select()

    def Spawn(self):
        self.children = {}

        for parent in self.parents.keys():
            self.children[parent] = copy.deepcopy(self.parents[parent])
            self.children[parent].Set_ID(self.nextAvailableID)
            self.nextAvailableID += 1

    def Mutate(self):
        for child in self.children.values():
            child.Mutate()

    def Evaluate(self, solutions):
        for solution in solutions.values():
            solution.Start_Simulation("DIRECT")

        for solution in solutions.values():
            solution.Wait_For_Simulation_To_End()

    def Select(self):
        for parent in self.parents.keys():
            if self.children[parent].fitness > self.parents[parent].fitness:
                #print(self.children[parent].fitness, self.parents[parent].fitness)
                self.parents[parent] = self.children[parent]

    def Print(self):
        for parent in self.parents.keys():
            print(f"parent={self.parents[parent].fitness} child={self.children[parent].fitness}")
        print()

    def Show_Best(self):
        bestParent = 0

        for parent in self.parents.keys():
            if self.parents[parent].fitness > self.parents[bestParent].fitness:
                bestParent = parent

        self.parents[bestParent].Start_Simulation("GUI", background=False)
        print(f"best fitness: {self.parents[bestParent].fitness}")
        print(f"best id: {self.parents[bestParent].myID}")

    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(name="Box", pos=[-3, 3, 0.5], size=[1, 1, 1])
        pyrosim.End()

    def Create_Body(self):
        pyrosim.Start_URDF("body.urdf")
        pyrosim.Send_Cube(name="Torso", pos=[0, 0, 1.0], size=[1.0, 1.0, 1.0])

        pyrosim.Send_Joint(name="Torso_BackLeg",
                           parent="Torso",
                           child="BackLeg", type="revolute",
                           position=[0, -0.5, 1.0],
                           jointAxis="1 0 0")
        pyrosim.Send_Cube(name="BackLeg",
                          pos=[0, -0.5, 0],
                          size=[0.2, 1.0, 0.2])

        pyrosim.Send_Joint(name="Torso_FrontLeg",
                           parent="Torso",
                           child="FrontLeg", type="revolute",
                           position=[0, 0.5, 1.0],
                           jointAxis="1 0 0")

        pyrosim.Send_Cube(name="FrontLeg",
                          pos=[0, 0.5, 0],
                          size=[0.2, 1.0, 0.2])

        pyrosim.Send_Joint(name="Torso_LeftLeg",
                           parent="Torso",
                           child="LeftLeg", type="revolute",
                           position=[-0.5, 0, 1.0],
                           jointAxis="0 1 0")

        pyrosim.Send_Cube(name="LeftLeg",
                          pos=[-0.5, 0, 0],
                          size=[1.0, 0.2, 0.2])

        pyrosim.Send_Joint(name="Torso_RightLeg",
                           parent="Torso",
                           child="RightLeg", type="revolute",
                           position=[0.5, 0, 1.0],
                           jointAxis="0 1 0")

        pyrosim.Send_Cube(name="RightLeg",
                          pos=[0.5, 0, 0],
                          size=[1.0, 0.2, 0.2])

        pyrosim.Send_Joint(name="BackLeg_BackLowerLeg",
                           parent="BackLeg",
                           child="BackLowerLeg", type="revolute",
                           position=[0, -1.0, 0],
                           jointAxis="1 0 0")

        pyrosim.Send_Cube(name="BackLowerLeg",
                          pos=[0, 0, -0.5],
                          size=[0.2, 0.2, 1.0])

        pyrosim.Send_Joint(name="FrontLeg_FrontLowerLeg",
                           parent="FrontLeg",
                           child="FrontLowerLeg", type="revolute",
                           position=[0, 1.0, 0],
                           jointAxis="1 0 0")

        pyrosim.Send_Cube(name="FrontLowerLeg",
                          pos=[0, 0, -0.5],
                          size=[0.2, 0.2, 1.0])

        pyrosim.Send_Joint(name="LeftLeg_LeftLowerLeg",
                           parent="LeftLeg",
                           child="LeftLowerLeg", type="revolute",
                           position=[-1.0, 0, 0],
                           jointAxis="0 1 0")

        pyrosim.Send_Cube(name="LeftLowerLeg",
                          pos=[0, 0, -0.5],
                          size=[0.2, 0.2, 1.0])

        pyrosim.Send_Joint(name="RightLeg_RightLowerLeg",
                           parent="RightLeg",
                           child="RightLowerLeg", type="revolute",
                           position=[1.0, 0, 0],
                           jointAxis="0 1 0")

        pyrosim.Send_Cube(name="RightLowerLeg",
                          pos=[0, 0, -0.5],
                          size=[0.2, 0.2, 1.0])

        pyrosim.End()
