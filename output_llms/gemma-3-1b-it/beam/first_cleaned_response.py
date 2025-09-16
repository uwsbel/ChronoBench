import pychrono
import numpy as np
import matplotlib.pyplot as plt
from pychrono.physics import BeamFiniteElement
from pychrono.visualize import visualize


num_nodes = 100
beam_length = 1.0
beam_width = 0.1
beam_thickness = 0.05
num_beams = 5
num_nodes_per_beam = 2

material_density = 1.0
material_viscosity = 0.01
material_conductivity = 100.0


mesh = BeamFiniteElement(
    num_nodes,
    beam_length,
    beam_width,
    beam_thickness,
    num_beams,
    num_nodes_per_beam,
    material_density,
    material_viscosity,
    material_conductivity,
    
)


initial_positions = np.random.rand(num_nodes, 2) * beam_length
initial_velocities = np.random.rand(num_nodes, 2) * 1.0


simulation_time = 10.0
time_step = 0.01
simulation_duration = 10.0


simulation = simulation_time
while simulation > 0:
    
    for i in range(num_nodes):
        
        force = np.zeros(2)
        for j in range(num_beams):
            
            force[j] = material_density * material_viscosity * material_conductivity * initial_positions[i, j] * (1.0 - initial_velocities[i, j])

        
        mesh.apply_force(force)

        
        initial_positions[i, :] = initial_positions[i, :] + initial_velocities[i, :] * time_step
        initial_velocities[i, :] = initial_velocities[i, :] + initial_velocities[i, :] * time_step

    
    simulation += time_step

    
    visualize(mesh, time=simulation, title="Beam Simulation")

    
    plt.pause(0.01)


simulation = simulation.stop()

print("Simulation complete.")