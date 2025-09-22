import pychrono
import pychrono.core as pc
import pychrono.visual as pv
import numpy as np
import matplotlib.pyplot as plt
from pychrono.visual import Camera
from pychrono.core import force
from pychrono.visual import Scene
from pychrono.visual import Light

# Define parameters
mass = 1.0  # kg
spring_constant = 10.0  # N/m
damping_coefficient = 0.01  # Ns/m
distance_to_ground = 0.5  # meters
simulation_time = 10.0  # seconds
gravity = 9.81  # m/s^2

# Create a scene
scene = Scene()
scene.add_object(pc.Mass(mass, 'mass'))
scene.add_object(pc.Spring(mass, spring_constant, damping_coefficient, distance_to_ground, gravity))
scene.add_object(pc.Ground(mass, spring_constant, damping_coefficient, distance_to_ground, gravity))

# Create a camera
camera = Camera(scene, 0.0, 0.0, 1.0)

# Set lighting
light = Light(camera, 0.0, 0.0, 1.0)

# Set visualization settings
pv.set_options(
    'title', 'Mass-Spring-Damper Simulation',
    'width', 800,
    'height', 600
)

# Set camera settings
camera.set_camera_settings(light)

# Run the simulation
simulation = pc.Simulation(scene, camera)
simulation.run(simulation_time)

# Save the simulation
simulation.save_simulation()

# Display the simulation
pv.show_simulation(scene)

# Optional:  Add a simple plot of the mass's position over time
# This is just for visual confirmation and isn't strictly required for the
# core functionality.
# plt.plot(simulation.time, simulation.mass.x)
# plt.xlabel("Time (s)")
# plt.ylabel("Mass (kg)")
# plt.title("Mass Position Over Time")
# plt.show()