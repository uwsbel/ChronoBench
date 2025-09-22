import pychrono
import pychrono.core as pc
import pychrono.visual as v
import numpy as np
import matplotlib.pyplot as plt
from pychrono.visual.primitive import Primitive
from pychrono.visual.rigidterrain import RigidTerrain
from pychrono.visual.driver import Driver
from pychrono.visual.visual import Visual

# --- Simulation Parameters ---
vehicle_id = 1  # Unique identifier for the vehicle
vehicle_type = "HMMWV"
terrain_width = 10.0
terrain_height = 5.0
terrain_texture = "grass"  # Default terrain texture
vehicle_speed = 1.0
vehicle_acceleration = 0.1
vehicle_deceleration = 0.1
vehicle_max_speed = 2.0
vehicle_max_acceleration = 1.0
vehicle_max_deceleration = 0.1
vehicle_contact_method = "rigid" # "rigid" or "soft"
vehicle_tmeasy_tire_model = "TMEASY" # Default tire model
vehicle_center_x = 0.0
vehicle_center_y = 0.0
vehicle_center_z = 0.0

# --- Initial Vehicle Setup ---
vehicle_location = (vehicle_center_x, vehicle_center_y, vehicle_center_z)
vehicle_orientation = (0.0, 0.0, 0.0)  # Initial orientation
vehicle_contact = pc.ContactMethod.rigid  # Default contact method
vehicle_tmeasy_tire = pc.TMEasy(vehicle_tmeasy_tire_model)

# --- Terrain Setup ---
terrain = RigidTerrain(width=terrain_width, height=terrain_height)
terrain.texture = terrain_texture
terrain.set_center(vehicle_location)

# --- Driver Setup ---
driver = Driver()
driver.set_vehicle_id(vehicle_id)
driver.set_vehicle_type(vehicle_type)
driver.set_terrain(terrain)
driver.set_contact(vehicle_contact)
driver.set_tmeasy_tire(vehicle_tmeasy_tire)

# --- Simulation Loop ---
def update_simulation():
    global vehicle_location, vehicle_orientation, vehicle_contact, vehicle_tmeasy_tire
    
    # Simple movement update (replace with a more sophisticated model)
    vehicle_location = (vehicle_location[0] + vehicle_speed * 0.1,
                       vehicle_location[1] + vehicle_speed * 0.1,
                       vehicle_location[2] + vehicle_speed * 0.1)

    vehicle_orientation = (vehicle_orientation[0], vehicle_orientation[1], vehicle_orientation[2])

    # Contact update
    vehicle_contact = vehicle_contact.rigid

    # Simple braking
    vehicle_contact.brake(vehicle_acceleration)
    
    # Visualize the vehicle
    v.draw_primitive(vehicle_id, vehicle_location, vehicle_orientation, vehicle_contact)

    # Update the simulation
    pc.update()
    
    # Update the simulation every frame
    clock = pc.Clock()
    
    # Print some information (optional)
    print(f"Frame: {clock.time()}")

# --- Main Execution ---
if __name__ == "__main__":
    update_simulation()
    
    # Run the simulation for 50 frames
    pc.run(update_simulation)