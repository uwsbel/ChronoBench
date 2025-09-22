import pychrono
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 1. Initialize PyChrono environment
try:
    chrono = pychrono.chrono.Chronos()
except Exception as e:
    print(f"Error initializing Chronos: {e}")
    exit()

# 2. Define the physical system - Beam
#   - ANCF cable elements
#   - Hinged end
#   - Gravity

# Define the beam parameters
beam_length = 1.0  # Length of the beam in meters
cable_radius = 0.05 # Radius of the cable elements
cable_thickness = 0.01 # Thickness of the cable elements
gravity = 9.81 # Acceleration due to gravity

# Define the number of cable elements
num_elements = 10

# Define the positions of the cable elements
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

# 3. Create the simulation object
simulation = chrono.create_simulation(
    num_elements=num_elements,
    cable_radius=cable_radius,
    cable_thickness=cable_thickness,
    gravity=gravity,
    # Add other parameters as needed (e.g., damping, boundary conditions)
)

# 4. Set default parameters (optional, but good practice)
simulation.set_default_parameters(
    cable_radius=cable_radius,
    cable_thickness=cable_thickness,
    gravity=gravity,
    damping=0.0, # Set damping to 0 for no damping
    boundary_conditions = chrono.BoundaryConditions.Fixed
)

# 5. Visualize the beam (using Matplotlib Animation)
fig, ax = plt.subplots()
ax.set_xlim(-beam_length/2, beam_length/2)
ax.set_ylim(-beam_length/2, -beam_length/2)
ax.set_aspect('equal')
ax.set_title("Beam Simulation")

def animate(i):
    cable_positions[i] = cable_positions[i] + np.array([0, 0, 0])
    # Update the animation
    ax.clear()
    ax.set_xlim(-beam_length/2, beam_length/2)
    ax.set_ylim(-beam_length/2, -beam_length/2)
    ax.set_aspect('equal')
    ax.set_title("Beam Deformation")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_style('ticks')
    return ax

# Create the animation
ani = FuncAnimation(fig, animate, frames=len(cable_positions), interval=20)

# 6. Display the animation
plt.show()

# 7. Save the animation (optional)
# plt.savefig("beam_simulation.gif")