import numpy as np
import matplotlib.pyplot as plt

class PLOT:
    def average_matrices(matrices):
        for matrix in matrices:
            if matrix.shape != matrices[0].shape:
                quit()

        sum_matrix = np.zeros(matrices[0].shape)
        for matrix in matrices:
            sum_matrix += matrix

        average_matrix = sum_matrix / len(matrices)

        return average_matrix
    
    def plot():
        # Load the fitness matrix from the .txt file
        fitness_matrixA1 = np.loadtxt("jump_matrixA1.txt")
        fitness_matrixA2 = np.loadtxt("jump_matrixA2.txt")
        fitness_matrixA3 = np.loadtxt("jump_matrixA3.txt")
        fitness_matrixA4 = np.loadtxt("jump_matrixA4.txt")
        fitness_matrixA5 = np.loadtxt("jump_matrixA5.txt")
        fitness_matrixA6 = np.loadtxt("jump_matrixA6.txt")
        fitness_matrixA7 = np.loadtxt("jump_matrixA7.txt")
        fitness_matrixA8 = np.loadtxt("jump_matrixA8.txt")
        fitness_matrixA9 = np.loadtxt("jump_matrixA9.txt")
        fitness_matrixA10 = np.loadtxt("jump_matrixA10.txt")

        #fitness_matrixA = PLOT.average_matrices([fitness_matrixA1, fitness_matrixA2, fitness_matrixA3, fitness_matrixA4, fitness_matrixA5, fitness_matrixA6])
        fitness_matrixA = np.loadtxt("fitness_matrixA.txt")

        fitness_matrixB1 = np.loadtxt("jump_matrixB1.txt")
        fitness_matrixB2 = np.loadtxt("jump_matrixB2.txt")
        fitness_matrixB3 = np.loadtxt("jump_matrixB3.txt")
        fitness_matrixB4 = np.loadtxt("jump_matrixB4.txt")
        fitness_matrixB5 = np.loadtxt("jump_matrixB5.txt")
        fitness_matrixB6 = np.loadtxt("jump_matrixB6.txt")
        fitness_matrixB7 = np.loadtxt("jump_matrixB7.txt")
        fitness_matrixB8 = np.loadtxt("jump_matrixB8.txt")
        fitness_matrixB9 = np.loadtxt("jump_matrixB9.txt")
        fitness_matrixB10 = np.loadtxt("jump_matrixB10.txt")

        #fitness_matrixB = PLOT.average_matrices([fitness_matrixB1, fitness_matrixB2, fitness_matrixB3, fitness_matrixB4, fitness_matrixB5, fitness_matrixB6, fitness_matrixB7,fitness_matrixB8,fitness_matrixB9,fitness_matrixB10])
        fitness_matrixB = np.loadtxt("fitness_matrixB.txt")

        # Get the number of solutions (rows) and generations (columns) in the fitness matrix
        num_solutions, num_generations = fitness_matrixA.shape
        mean_fitness_A = np.mean(fitness_matrixA, axis=0)
        mean_fitness_B = np.mean(fitness_matrixB, axis=0)

        # Plot the mean fitness values for both matrices
        generations = np.arange(1, num_generations + 1)
        plt.plot(generations, mean_fitness_A, label="Fitness Matrix A", linewidth=3)
        plt.plot(generations, mean_fitness_B, label="Fitness Matrix B")

        plt.xlabel("Generation")
        plt.ylabel("Fitness")
        plt.title("Fitness Values for Each Generation")
        plt.legend()
        plt.grid(True)
        plt.show()
#PLOT.plot()

