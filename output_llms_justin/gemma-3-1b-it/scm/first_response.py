import pychrono
import pychrono.core as chrono
import pychrono.visual as visual
import numpy as np
import matplotlib.pyplot as plt
from pychrono.visual.mesh import Mesh
from pychrono.visual.renderer import Renderer
from pychrono.visual.scene import Scene

# --- Simulation Parameters ---
vehicle_id = 1
vehicle_type = "HMMWV"
terrain_type = "SCM"
terrain_height = 0.1  # meters
terrain_slope = 0.02  # meters per second^2
vehicle_speed = 1.0  # m/s
simulation_duration = 60  # seconds
frame_rate = 50

# --- Initial Vehicle Setup ---
vehicle_position = (0, 0, 0)  # meters
vehicle_orientation = (0, 0, 0)  # radians
vehicle_rigid_tire_model = {
    "tire": {"radius": 0.2, "height": 0.05},
    "suspension": {"spring_rate": 0.05, "damping": 0.1}
}

# --- SCM Terrain Setup ---
terrain_patch_size = 10.0  # meters
terrain_patch_height = 0.05 # meters
terrain_slope_factor = 0.02 # meters/second^2
terrain_soil_parameters = {
    "water_level": 0.0,
    "soil_density": 0.001
}

# --- Simulation Setup ---
simulation = chrono.Simulation(vehicle_id, vehicle_type, terrain_type,
                              vehicle_position, vehicle_orientation,
                              vehicle_rigid_tire_model,
                              terrain_patch_size, terrain_patch_height,
                              terrain_slope_factor, terrain_soil_parameters)

# --- Visualization ---
renderer = Renderer(vehicle_type=vehicle_type,
                    vehicle_position=vehicle_position,
                    vehicle_orientation=vehicle_orientation,
                    vehicle_rigid_tire_model=vehicle_rigid_tire_model,
                    terrain_patch_size=terrain_patch_size,
                    terrain_patch_height=terrain_patch_height)
scene = Scene(renderer=renderer)

# --- Real-Time Simulation Loop ---
for frame in range(simulation.frames):
    # --- Update Vehicle Dynamics ---
    vehicle_position += vehicle_speed * frame * 0.01  # Adjust speed for smoother simulation
    vehicle_orientation += vehicle_speed * frame * 0.01  # Adjust speed for smoother simulation

    # --- Terrain Update ---
    terrain_patch_height += terrain_slope_factor * frame * 0.01
    terrain_patch_height = max(0.0, terrain_patch_height) # Ensure height doesn't go negative

    # --- Rendering ---
    scene.render(renderer)
    scene.update()
    scene.display()

    # --- Print Frame Information ---
    print(f"Frame {frame}: {vehicle_position}, {vehicle_orientation}")

    # --- Simulation Timer ---
    simulation.update()
    simulation.step()

    # --- Optional:  Add a delay for visual clarity (adjust as needed) ---
    #time.sleep(0.01)  # Adjust for desired visual smoothness

# --- Simulation Completion ---
simulation.end()