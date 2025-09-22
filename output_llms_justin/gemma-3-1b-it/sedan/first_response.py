import pychrono
import pychrono.core as pc
import pychrono.visual as pv
import irlicht as il
import numpy as np
import matplotlib.pyplot as plt

# --- Simulation Parameters ---
terrain_size = 100  # meters
vehicle_mass = 1500  # kg
vehicle_speed = 2.0  # m/s
tire_radius = 0.3  # meters
tire_pressure = 1.0  # kPa
acceleration = 0.2  # m/s^2
brake_force = 0.1  # N
simulation_time = 60  # seconds

# --- Initial Setup ---
pv.init(title="BMW E90 Sedan Simulation",  # Title of the simulation
          width=800,  # Width of the visualization
          height=600,  # Height of the visualization
          camera_fov=60,  # Field of view of the camera
          camera_resolution=100)

# --- Terrain Generation (Simple - can be expanded) ---
terrain = pc.Terrain(
    size=terrain_size,
    height=terrain_size * 0.8,  # Terrain height
    # Add more terrain features here (e.g., hills, valleys)
    # This is a placeholder - replace with a more realistic terrain generation
    # method
)

# --- Vehicle Setup ---
vehicle = pc.Vehicle(
    mass=vehicle_mass,
    speed=vehicle_speed,
    tire_radius=tire_radius,
    tire_pressure=tire_pressure,
    acceleration=acceleration,
    brake_force=brake_force,
    orientation=pc.Orientation(
        x=0.0,
        y=0.0,
        z=0.0
    )
)

# --- Collision Settings ---
collision_settings = pc.CollisionSettings(
    collision_type='rigid',
    collision_threshold=0.1  # meters
)

# --- Visualization ---
pv.render(
    vehicle,
    terrain,
    camera,
    collision_settings,
    simulation_time,
    pv.RenderingMode.INSIDE
)


# --- Simulation Loop ---
for t in range(simulation_time):
    # Update Vehicle Position
    vehicle.update(t)

    # Update Terrain
    terrain.update(t)

    # Update Visualization
    pv.render(
        vehicle,
        terrain,
        camera,
        collision_settings,
        simulation_time,
        pv.RenderingMode.OUTSIDE
    )

    # Print Simulation Information (for debugging)
    print(f"Time: {t}")

    # Optional:  Add a delay to control the simulation speed
    # time.sleep(0.01) # Adjust delay as needed

# --- Cleanup ---
#  If you want to save the simulation to a file
#  pv.save("bmw_simulation.png")