import pychrono
import numpy as np
import matplotlib.pyplot as plt
from pychrono.core import System
from pychrono.core.physics import Physics
from pychrono.core.visual import Visual

def simulate_beam_finite_element(num_nodes=100, beam_length=1.0, beam_thickness=0.1, force_magnitude=1.0, damping=0.0):
    """
    Simulates a beam finite element system using PyChrono.

    Args:
        num_nodes (int): Number of nodes in the mesh.
        beam_length (float): Length of the beam.
        beam_thickness (float): Thickness of the beam.
        force_magnitude (float): Magnitude of the applied force.
        damping (float): Damping coefficient.
    """

    # 1. Initialize PyChrono environment
    env = System(num_nodes=num_nodes,  # Number of nodes
                    beam_length=beam_length,
                    beam_thickness=beam_thickness,
                    force_magnitude=force_magnitude,
                    damping=damping)

    # 2. Define the physical system
    physics = Physics(env)

    # 3. Define the mesh (a simple rectangular beam)
    mesh = np.zeros((num_nodes, 2))
    mesh[0, 0] = 1.0  # Define the starting point of the beam
    mesh[num_nodes - 1, 0] = 1.0
    mesh[0, 1] = 1.0
    mesh[num_nodes - 1, 1] = 1.0

    # 4. Set the initial conditions
    initial_conditions = np.array([
        mesh[0, 0], mesh[0, 1], mesh[1, 0], mesh[1, 1],
        mesh[2, 0], mesh[2, 1], mesh[3, 0], mesh[3, 1]
    ])

    # 5. Run the simulation
    env.run(physics, initial_conditions)

    # 6. Visualize the results
    plt.figure(figsize=(8, 6))
    plt.imshow(mesh, cmap='viridis', origin='lower')
    plt.title("Beam Finite Element Simulation")
    plt.colorbar(label="Strain")
    plt.show()

if __name__ == "__main__":
    simulate_beam_finite_element()