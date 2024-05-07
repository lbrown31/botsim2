from parallelHillClimber import PARALLEL_HILL_CLIMBER
from plotFitnessValues import PLOT

phc = PARALLEL_HILL_CLIMBER()
phc.Evolve()
phc.Save_Fitness_Matrix("fitness_matrixB.txt")
phc.Save_Jump_Matrix("jump_matrixB1.txt")

phc.Show_Best()
#PLOT.plot()