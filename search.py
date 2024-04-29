from parallelHillClimber import PARALLEL_HILL_CLIMBER
from plotFitnessValues import PLOT

phc = PARALLEL_HILL_CLIMBER()
phc.Evolve()
phc.Save_Fitness_Matrix("fitness_matrixB.npy")
phc.Show_Best()
PLOT.plot()