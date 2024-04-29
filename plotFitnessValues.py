import numpy as np
import matplotlib.pyplot as plt

class PLOT:
    def plot():
        # Load the fitness matrix from the .npy file
        fitness_matrix = np.load("fitness_matrixB.npy")

        # Get the number of solutions (rows) and generations (columns) in the fitness matrix
        num_solutions, num_generations = fitness_matrix.shape

        # Plot a curve for each row (solution) of the fitness matrix
        for i in range(num_solutions):
            plt.plot(fitness_matrix[i, :], label=f"Solution {i+1}")

        plt.xlabel("Generation")
        plt.ylabel("Fitness")
        plt.title("Fitness Values for Each Solution")
        plt.legend()
        plt.grid(True)
        plt.show()