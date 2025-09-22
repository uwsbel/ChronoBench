import time
import numpy as np
from pychrono import Chrono, System
from pychrono.utils import get_system_properties

# --- Simulation Parameters ---
gravity = 9.81  # m/s^2
timestep = 0.01  # seconds
simulation_duration = 10  # seconds
max_steps = 100

# --- System Setup ---
system = System()
system.set_properties(get_system_properties())  # Initialize system properties

# --- Physical Systems ---
floor = System.Floor()
floor.set_position(0, 0, 0)  # Initial position

# --- Movable Box ---
box = System.Box()
box.set_position(5, 0, 0)  # Initial position
box.set_mass(10.0)  # Mass of the box (adjust as needed)

# --- Initial Conditions ---
# Initial position of the floor
floor.set_position(0, 0, 0)

# Initial position of the box
box.set_position(5, 0, 0)

# --- Simulation Loop ---
for step in range(simulation_duration):
    # Update the system state
    system.update()

    # Update the floor's position
    floor.set_position(0, 0, step * gravity)

    # Update the box's position
    box.set_position(5, 0, step * gravity)

    # Publish a message to the ROS topic
    ros_message = "step_" + str(step)
    print(f"Step {step}: Publishing {ros_message}")
    # Add a delay to control simulation speed.  Adjust as needed.
    time.sleep(timestep)

print("Simulation complete.")