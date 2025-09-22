import pychrono
import pychrono.core as pc
import pychrono.visuals as v
import pychrono.physics as ph
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm

# --- Configuration ---
# Define the truck model
truck_model = pc.VehicleModel(
    name="Truck",
    length=10,
    mass=1000,
    acceleration=0.5,
    turning_radius=0.5,
    max_speed=10,
    steering_angle_min=0,
    steering_angle_max=180,
    brake_force=0.1,
    visual_settings={"colormap": cm.gray},
    collision_settings={"collision_threshold": 0.1}
)

# Define the TMEASY tire model
tmeasy_tire = pc.TMEASY(
    model_name="TMEASY",
    radius=0.3,
    height=0.2,
    friction=0.9,
    mass=1500,
    dynamic_friction=0.05,
    visual_settings={"colormap": cm.blue}
)

# Define the terrain
terrain = pc.Terrain(
    name="Terrain",
    width=100,
    height=50,
    texture="grass",
    visual_settings={"resolution": 100}
)

# Define the camera settings
camera_settings = pc.Camera(
    name="Camera",
    width=800,
    height=600,
    zoom=1.0,
    focal_length=1000,
    camera_fov=60,
    visual_settings={"rotation": 0.0}
)

# --- Simulation Parameters ---
simulation_time = 60  # seconds
time_step = 0.1  # seconds
max_simulation_time = 60  # seconds

# --- Initialization ---
simulation = pc.Simulation(
    name="Truck_Simulation",
    time=simulation_time,
    time_step=time_step,
    max_simulation_time=max_simulation_time,
    vehicle=truck_model,
    terrain=terrain,
    camera=camera_settings,
    physics=ph.Physics(
        mass=truck_model.mass,
        acceleration=truck_model.acceleration,
        turning_radius=truck_model.turning_radius,
        max_speed=truck_model.max_speed,
        steering_angle_min=truck_model.steering_angle_min,
        steering_angle_max=truck_model.steering_angle_max,
        brake_force=truck_model.brake_force,
        collision_settings=truck_model.collision_settings
    )
)

# --- Visualization Setup ---
v.init_visuals()
v.set_camera(camera_settings)

# --- Simulation Loop ---
for i in range(simulation_time):
    # 1. Vehicle Dynamics
    vehicle_state = simulation.vehicle.state
    vehicle_state['position'] = vehicle_state['position'] + np.array(vehicle_state['velocity']) * time_step
    vehicle_state['velocity'] = vehicle_state['velocity'] + np.array(vehicle_state['acceleration']) * time_step

    # 2. TMEASY Tire Dynamics
    tmeasy_state = simulation.vehicle.state
    tmeasy_state['position'] = tmeasy_state['position'] + np.array(tmeasy_state['velocity']) * time_step
    tmeasy_state['velocity'] = tmeasy_state['velocity'] + np.array(tmeasy_state['acceleration']) * time_step

    # 3. Terrain Interaction
    terrain_state = simulation.terrain.state
    terrain_state['position'] = terrain_state['position'] + np.array(terrain_state['velocity']) * time_step

    # 4. Visualization
    v.draw_object(camera_settings, vehicle_state)
    v.draw_object(camera_settings, tmeasy_state)
    v.draw_object(camera_settings, terrain_state)

    # 5. Print Simulation Status
    print(f"Time: {i*time_step:.2f} s")

# --- End of Simulation ---
simulation.end()