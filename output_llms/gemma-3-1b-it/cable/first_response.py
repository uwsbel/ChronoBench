import pychrono
import pychrono.core as pc
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Define parameters
beam_length = 1.0  # Length of the beam in meters
cable_radius = 0.05  # Radius of the cable elements
gravity = 9.81  # Acceleration due to gravity
cable_strength = 1000.0  # Strength of the cable elements (adjust as needed)
num_elements = 10  # Number of cable elements
initial_position = [0.0, 0.0]  # Initial position of each cable element
initial_velocity = 0.0  # Initial velocity of each cable element (for dynamic simulation)
# Create PyChrono environment
env = pc.Environment()

# Create the beam
beam = env.create_beam(beam_length, cable_radius, cable_strength)

# Set initial positions
for i in range(num_elements):
    beam[i].position = initial_position[i]

# Set initial velocities (for dynamic simulation)
for i in range(num_elements):
    beam[i].velocity = initial_velocity[i]

# Set the simulation parameters
env.set_time_step(0.001)  # Time step for the simulation
env.set_simulation_time(0, 100)  # Simulation duration (100 seconds)
env.set_physics_parameters(
    physics_type="static",
    gravity=gravity,
    cable_strength=cable_strength,
    beam_length=beam_length
)

# Simulation loop
def update(dt):
    """Updates the simulation for one time step."""
    for i in range(num_elements):
        # Calculate forces and moments
        force = np.array([0.0, 0.0, 0.0])
        moment = np.array([0.0, 0.0, 0.0])

        # Calculate forces due to cable elements
        for j in range(num_elements):
            if i != j:
                force[i] += beam[i].velocity[j] * cable_strength * (beam[i].position[j] - beam[i].position[j])
                moment[i] += beam[i].velocity[j] * cable_strength * (beam[i].position[j] - beam[i].position[j])

        # Apply forces and moments to the beam
        beam[i].velocity[i] += force[i] * dt
        beam[i].moment[i] += force[i] * dt

        # Update position
        beam[i].position[i] += beam[i].velocity[i] * dt

    return beam

# Create the animation
fig, ax = plt.subplots()
ax.set_xlim(0, beam_length)
ax.set_ylim(0, beam_length)
ax.set_aspect('equal')
ax.set_title("Beam Simulation")
ax.set_xlabel("X")
ax.set_ylabel("Y")

# Create the beam object
beam_obj = pc.Beam(beam, cable_strength)

# Animation function
def animate(i):
    """Updates the animation for each frame."""
    beam_obj.update(dt)
    return beam_obj

# Create the animation
ani = animation.FuncAnimation(fig, animate, frames=100, blit=True, interval=dt*1000/1000)  # Update every 100ms

plt.show()