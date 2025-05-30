import pychrono
import pychrono.core as pc
import pychrono.visuals as v
import pychrono.physics as ph
import pychrono.utils as u
import numpy as np
import matplotlib.pyplot as plt  # For visualization (optional, but recommended)

# --- Simulation Parameters ---
terrain_width = 10.0
terrain_height = 5.0
terrain_depth = 2.0
terrain_scale = 10.0
vehicle_width = 2.0
vehicle_length = 3.0
vehicle_mass = 100.0
vehicle_friction = 0.8
vehicle_tire_radius = 0.5
vehicle_tire_pressure = 0.7  # PSI
vehicle_tmeasy_model = "tmeasy_model_2" # Adjust as needed
# --- Initial Setup ---
# 1. Initialize PyChrono
chrono = pc.Chronos()

# 2. Create Terrain
terrain = u.Terrain(
    width=terrain_width,
    height=terrain_height,
    depth=terrain_depth,
    scale=terrain_scale,
    texture="grass.png" # Replace with your terrain texture
)

# 3. Create Vehicle
vehicle = pc.Vehicle(
    width=vehicle_width,
    length=vehicle_length,
    mass=vehicle_mass,
    friction=vehicle_friction,
    tmeasy_model=vehicle_tmeasy_model,
    position=(0, 0, 0) # Initial position
)

# 4. Set Initial Forces & Interactions
# 5.  RigidTerrain Setup
rigid_terrain = pc.RigidTerrain(
    width=terrain_width,
    height=terrain_height,
    depth=terrain_depth,
    scale=terrain_scale,
    texture="grass.png",
    # Add any other terrain properties here (e.g., slope, elevation)
)


# --- Simulation Loop ---
try:
    # 6.  Initialize Chrono for Simulation
    chrono.init(chrono)

    # 7.  Simulation Loop (50 FPS)
    while True:
        # 8.  Update Vehicle Dynamics
        vehicle.update_dynamics()

        # 9.  Update Terrain
        rigid_terrain.update_terrain()

        # 10.  Render
        v.render(vehicle, rigid_terrain,  ) # Use v.render for better rendering

        # 11.  Print Status (optional - for debugging)
        print(f"Time: {chrono.time()}")

        # 12.  Wait for a short time to allow rendering
        chrono.wait(1)  # Short delay to allow rendering

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    # 13. Clean Up
    chrono.cleanup()
    print("Simulation finished.")