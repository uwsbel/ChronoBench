import pychrono
import numpy as np
import matplotlib.pyplot as plt
from pychrono.simulation import Simulation

# 1. Initialize PyChrono Environment
try:
    simulation = Simulation()
except Exception as e:
    print(f"Error initializing PyChrono: {e}")
    exit()

# 2. Define Physical Systems and Objects
# Particles:  We'll use a simple sphere for demonstration.
particle_type = "sphere"
particle_radius = 0.1
particle_mass = 1.0  # kg
particle_velocity = np.array([1.0, 0.5, 0.0])  # m/s
particle_orientation = np.array([0.0, 0.0, 1.0])  # radians -  pointing along the z-axis
particle_color = "red"

# Gravitational Source:  A simple point mass.
gravitational_source_mass = 10.0  # kg
gravitational_source_position = np.array([0.0, 0.0, 0.0])
gravitational_source_force = 0.0  # N/kg

# 3. Set Default Parameters
simulation.set_default_parameters()
simulation.set_particle_type(particle_type)
simulation.set_particle_mass(particle_mass)
simulation.set_particle_velocity(particle_velocity)
simulation.set_particle_orientation(particle_orientation)
simulation.set_particle_color(particle_color)
simulation.set_gravitational_source_mass(gravitational_source_mass)
simulation.set_gravitational_source_position(gravitational_source_position)
simulation.set_gravitational_source_force(gravitational_source_force)


# 4. Add Particles to the Simulation
for i in range(100):  # Simulate 100 particles
    # Generate a random position and orientation
    position = np.random.rand(3) * simulation.get_range_min() + simulation.get_range_min()
    orientation = np.random.rand(3) * simulation.get_range_min() + simulation.get_range_min()

    # Create a particle object
    particle = Simulation.Particle(
        particle_type=particle_type,
        particle_radius=particle_radius,
        particle_mass=particle_mass,
        particle_velocity=particle_velocity,
        particle_orientation=particle_orientation,
        particle_color=particle_color
    )

    # Add the particle to the simulation
    simulation.add_particle(particle)


# 5. Visualization (using Irrlicht - requires matplotlib)
try:
    plt.ion()  # Enable interactive mode
    fig, ax = plt.subplots()
    ax.set_aspect('equal')
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title("Particle Simulation")

    for particle in simulation.particles:
        ax.plot(particle.position[0], particle.position[1], 'b-', linewidth=0.5)  # Blue line for the particle
        ax.text(particle.position[0] + particle.radius / 2, particle.position[1] + particle.radius / 2,
                f"{particle.color} ({particle.mass:.2f})", size=10) # Display particle info

    plt.show()
except Exception as e:
    print(f"Error during visualization: {e}")
finally:
    simulation.stop() # Important to stop the simulation