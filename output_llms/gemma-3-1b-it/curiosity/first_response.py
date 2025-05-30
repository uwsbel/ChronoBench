import pychrono
import pychrono.core as pc
import pychrono.visual as v
import numpy as np
import matplotlib.pyplot as plt  # For visualization (optional, but recommended)

# 1.1. Environment Setup
# Define the terrain (a simple grid)
terrain = pc.Terrain(
    width=100,
    height=100,
    grid_size=(5, 5), # Adjust for desired terrain detail
    # Add some ground texture (optional - a simple gray texture)
    ground_texture='gray'
)

# 2. Rover Model & Control
rover = pc.Rover(
    position=[0, 0, 0],  # Initial position
    orientation=[0, 0, 0], # Initial orientation
    motor_control_driver=pc.MotorControlDriver(
        max_torque=10, # Adjust for rover's capabilities
        max_speed=5,  # Adjust for rover's capabilities
        throttle=0.5,  # Adjust for rover's capabilities
        steering_sensitivity=0.1 # Adjust for rover's capabilities
    )
)

# 3. Initial Parameters
# Initial positions and velocities
rover.position = [10, 10, 0]
rover.orientation = [0, 0, 0]
rover.velocity = [0, 0, 0]
rover.motor_control_driver.set_throttle(0.5) #Set initial throttle

# 4. Visualization Setup
# Create the visualization
visual = v.Visualization(
    width=800,
    height=600,
    camera_resolution=100,  # Adjust for desired camera detail
    background_color='white',
    # Add some lighting (optional)
    lighting=pc.Lighting(
        color=pc.Color('white'),
        intensity=0.8,
    )
)

# 5.  Simulation Parameters
simulation_params = {
    'time_step': 0.1,  # Adjust for accuracy vs. speed
    'max_time': 60, # Simulation duration
    'simulation_type': pc.SimulationType.SIMPLE, # or 'SIMPLE', 'POLYGON'
    'gravity': pc.Gravity(
        acceleration=pc.Acceleration(0, -9.81), # Earth's gravity
        strength=9.81
    )
}


# 6.  Run the Simulation
simulation = pc.Simulation(
    simulation_params=simulation_params,
    environment=terrain,
    rover=rover,
    visual=visual
)

simulation.run()

# 7.  Visualization and Output
simulation.display()
# Optionally, save the visualization to a file:
# simulation.save_visualization('curiosity_simulation.png')