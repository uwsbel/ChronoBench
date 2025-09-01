import pychrono
import pychrono.physics as pc
import pychrono.visual as cv
import numpy as np
import matplotlib.pyplot as plt
from pychrono.simulation import Simulation

# 1. Environment Setup
# Define the simulation domain (e.g., a rectangular box)
domain = pc.Domain(
    dimensions=(10, 10, 10),  # Width, Length, Height
    # Add some random noise to the domain for more realistic behavior
    noise_level=0.1,
    # Define the initial conditions (e.g., initial position and velocity)
    initial_position=(0, 0, 0),
    initial_velocity=(1, 0, 0)
)

# 2. Physical Systems & Objects
# Jeffcott Rotor
rotor_model = pc.Model(
    name="JeffcottRotor",
    # Define the rotor geometry (simplified for this example)
    geometry=pc.Beam(
        length=1.0,
        width=0.5,
        height=0.5,
        # Add some random rotation
        rotation_angle=np.pi/2, # 90 degrees
        # Add some damping (simplified)
        damping=0.1
    ),
    # Define the rotor's mass and inertia
    mass=1.0,
    inertia=1.0,
    # Define the rotor's rotational speed (Hz)
    rotation_speed=10.0,
    # Define the rotor's damping coefficient
    damping_coefficient=0.05
)

# Flywheel
flywheel_model = pc.Model(
    name="Flywheel",
    # Define the flywheel geometry
    geometry=pc.Beam(
        length=1.0,
        width=0.5,
        height=0.5,
        # Add some random rotation
        rotation_angle=np.pi/2,
        # Define the flywheel's mass
        mass=1.0,
        # Define the flywheel's inertia
        inertia=1.0,
        # Define the flywheel's rotational speed (Hz)
        rotation_speed=10.0,
        # Define the flywheel's damping coefficient
        damping_coefficient=0.05
    ),
    # Define the flywheel's position
    position=(0, 0, 0)
)

# 3. Default Parameters
# Set default parameters for the simulation
simulation = pc.Simulation(
    domain=domain,
    physics_system=pc.System(rotor_model, flywheel_model),
    # Set initial conditions
    initial_position=domain.initial_position,
    initial_velocity=domain.initial_velocity,
    # Set simulation parameters
    simulation_parameters={"timestep": 0.01}
)

# 4. Visualization Setup
# Create a visualization window
cv.create_visualization(
    domain=domain,
    simulation=simulation,
    # Add a window title
    title="Jeffcott Rotor Simulation",
    # Customize the window appearance (optional)
    window_width=600,
    window_height=400,
    # Add a background color
    background_color=(0, 0, 0),
    # Add a grid
    grid_size=10,
    # Add a color bar
    color_bar_color=(1, 1, 1)
)


# 5. Simulation Loop
# Start the simulation
simulation.run()

# 6. Post-Processing (Optional)
# Print simulation parameters
print(simulation.get_parameters())

# Save the simulation (optional)
# simulation.save_simulation("jeffcott_rotor_simulation.pse")

# Display the simulation
cv.show_simulation()