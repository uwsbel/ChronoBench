import pychrono
import numpy as np
import matplotlib.pyplot as plt
from pychrono.environment import System
from pychrono.environment import Terrain
from pychrono.visual import Irrlicht
import time

# --- Configuration ---
simulation_name = "HMMWV Simulation"
simulation_time = 10  # Simulation duration in seconds
initial_position = [0, 0, 0]  # Initial vehicle position (x, y, z)
initial_velocity = [0, 0, 0]  # Initial vehicle velocity (x, y, z)
initial_force = [0, 0, 0]  # Initial force applied to the vehicle (x, y, z)
terrain_heightmap_path = "terrain.png"  # Replace with your terrain image path
terrain_heightmap_size = 100 # Size of the terrain heightmap in pixels

# --- System Setup ---
system = System(simulation_name, initial_position, initial_velocity, initial_force)
system.set_physics_system(pychrono.physics_system.Fluid) # Using Fluid for better realism
system.set_gravity(0) # Apply gravity
system.set_simulation_time(simulation_time)


# --- Terrain Setup ---
terrain = Terrain(
    initial_position,
    terrain_heightmap_path,
    terrain_heightmap_size,
    # Add more terrain parameters as needed (e.g., slope, roughness)
    # Example:  terrain_slope = 0.1  # Example slope
)

# --- Driver System ---
driver = Driver(system, "Driver")
driver.set_speed(initial_velocity) # Set initial speed
driver.set_acceleration(0) # Set initial acceleration

# --- Irrlicht Visualization ---
irrlicht = Irrlicht.initialize()
irrlicht.set_color("blue")
irrlicht.set_width(800)
irrlicht.set_height(600)
irrlicht.set_title(simulation_name)
irrlicht.rotate(np.pi / 2)  # Rotate the scene for better viewing

# --- Visualization Loop ---
def update_simulation():
    global system, terrain, driver, irrlicht

    # Update vehicle position
    system.update()
    vehicle_position = system.get_position()
    vehicle_velocity = system.get_velocity()
    vehicle_force = system.get_force()

    # Update terrain
    terrain.update()

    # Update driver
    driver.update()

    # Update Irrlicht
    irrlicht.clear()
    irrlicht.set_aspect(1)  # Maintain aspect ratio
    irrlicht.set_camera_position(vehicle_position)
    irrlicht.set_camera_look_at(terrain.get_center())

    # Draw the terrain
    irrlicht.plot(terrain.get_heightmap())

    # Draw the vehicle
    irrlicht.plot(vehicle_position)
    irrlicht.plot(vehicle_velocity)
    irrlicht.plot(vehicle_force)

    irrlicht.pause(0.01) # Small delay for visualization


# --- Main Simulation Loop ---
try:
    while True:
        update_simulation()
        time.sleep(0.1)  # Adjust for desired simulation speed

except KeyboardInterrupt:
    print("Simulation interrupted.")
    sys.exit()