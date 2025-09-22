import pychrono
import numpy as np
import matplotlib.pyplot as plt
import random

def simulate_gravity(num_particles, initial_positions, initial_velocities, initial_orientations):
    """
    Simulates gravitational attraction between particles.

    Args:
        num_particles (int): The number of particles to simulate.
        initial_positions (np.ndarray): A 2D numpy array representing the initial positions of the particles.
        initial_velocities (np.ndarray): A 2D numpy array representing the initial velocities of the particles.
        initial_orientations (np.ndarray): A 2D numpy array representing the initial orientations of the particles.
    """

    # Initialize PyChrono environment
    env = pychrono.Environment()

    # Create the simulation
    env.create_simulation(num_particles, initial_positions, initial_velocities, initial_orientations)

    # Simulation loop
    for i in range(num_particles):
        # Update particle positions and velocities
        env.update(i)

    # Visualize the simulation
    plt.figure(figsize=(8, 6))
    plt.imshow(env.get_state(), cmap='viridis')
    plt.title("Gravitational Simulation")
    plt.colorbar()
    plt.show()


if __name__ == '__main__':
    # Simulation parameters
    num_particles = 100
    initial_positions = np.random.rand(num_particles, 2) * 10  # Random positions in a 10x10 grid
    initial_velocities = np.random.rand(num_particles, 2) * 0.1  # Random velocities
    initial_orientations = np.random.rand(num_particles, 2) * np.pi / 180  # Random initial orientations

    # Run the simulation
    simulate_gravity(num_particles, initial_positions, initial_velocities, initial_orientations)