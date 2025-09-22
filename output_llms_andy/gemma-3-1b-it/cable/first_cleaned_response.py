import pychrono
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


try:
    chrono = pychrono.chrono.Chronos()
except Exception as e:
    print(f"Error initializing Chronos: {e}")
    exit()







beam_length = 1.0  
cable_radius = 0.05 
cable_thickness = 0.01 
gravity = 9.81 


num_elements = 10


cable_positions = np.array([
    [0, 0, 0],
    [cable_radius * 0.8, 0, 0],
    [cable_radius * 0.6, cable_radius * 0.8, 0],
    [0, cable_radius * 0.6, cable_radius * 0.8],
    [0, cable_radius * 0.8, cable_radius * 0.6],
    [cable_radius * 0.7, cable_radius * 0.8, cable_radius * 0.6],
    [0, cable_radius * 0.8, cable_radius * 0.6],
    [cable_radius * 0.6, cable_radius * 0.8, cable_radius * 0.6],
    [0, cable_radius * 0.6, cable_radius * 0.6],
    [cable_radius * 0.6, cable_radius * 0.6, cable_radius * 0.6]
])


simulation = chrono.create_simulation(
    num_elements=num_elements,
    cable_radius=cable_radius,
    cable_thickness=cable_thickness,
    gravity=gravity,
    
)


simulation.set_default_parameters(
    cable_radius=cable_radius,
    cable_thickness=cable_thickness,
    gravity=gravity,
    damping=0.0, 
    boundary_conditions = chrono.BoundaryConditions.Fixed
)


fig, ax = plt.subplots()
ax.set_xlim(-beam_length/2, beam_length/2)
ax.set_ylim(-beam_length/2, -beam_length/2)
ax.set_aspect('equal')
ax.set_title("Beam Simulation")

def animate(i):
    cable_positions[i] = cable_positions[i] + np.array([0, 0, 0])
    
    ax.clear()
    ax.set_xlim(-beam_length/2, beam_length/2)
    ax.set_ylim(-beam_length/2, -beam_length/2)
    ax.set_aspect('equal')
    ax.set_title("Beam Deformation")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_style('ticks')
    return ax


ani = FuncAnimation(fig, animate, frames=len(cable_positions), interval=20)


plt.show()