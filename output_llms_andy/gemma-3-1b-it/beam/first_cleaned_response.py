import pychrono
import numpy as np
import matplotlib.pyplot as plt
from pychrono.core import System
from pychrono.core.physics import Physics
from pychrono.core.visual import Visual

def simulate_beam_finite_element(num_nodes=100, beam_length=1.0, beam_thickness=0.1, force_magnitude=1.0, damping=0.0):
    

    
    env = System(num_nodes=num_nodes,  
                    beam_length=beam_length,
                    beam_thickness=beam_thickness,
                    force_magnitude=force_magnitude,
                    damping=damping)

    
    physics = Physics(env)

    
    mesh = np.zeros((num_nodes, 2))
    mesh[0, 0] = 1.0  
    mesh[num_nodes - 1, 0] = 1.0
    mesh[0, 1] = 1.0
    mesh[num_nodes - 1, 1] = 1.0

    
    initial_conditions = np.array([
        mesh[0, 0], mesh[0, 1], mesh[1, 0], mesh[1, 1],
        mesh[2, 0], mesh[2, 1], mesh[3, 0], mesh[3, 1]
    ])

    
    env.run(physics, initial_conditions)

    
    plt.figure(figsize=(8, 6))
    plt.imshow(mesh, cmap='viridis', origin='lower')
    plt.title("Beam Finite Element Simulation")
    plt.colorbar(label="Strain")
    plt.show()

if __name__ == "__main__":
    simulate_beam_finite_element()