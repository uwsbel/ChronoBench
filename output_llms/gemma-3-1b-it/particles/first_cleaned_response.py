import pychrono
import numpy as np
import matplotlib.pyplot as plt
import random

def simulate_gravity(num_particles, initial_positions, initial_velocities, initial_orientations):
    

    
    env = pychrono.Environment()

    
    env.create_simulation(num_particles, initial_positions, initial_velocities, initial_orientations)

    
    for i in range(num_particles):
        
        env.update(i)

    
    plt.figure(figsize=(8, 6))
    plt.imshow(env.get_state(), cmap='viridis')
    plt.title("Gravitational Simulation")
    plt.colorbar()
    plt.show()


if __name__ == '__main__':
    
    num_particles = 100
    initial_positions = np.random.rand(num_particles, 2) * 10  
    initial_velocities = np.random.rand(num_particles, 2) * 0.1  
    initial_orientations = np.random.rand(num_particles, 2) * np.pi / 180  

    
    simulate_gravity(num_particles, initial_positions, initial_velocities, initial_orientations)