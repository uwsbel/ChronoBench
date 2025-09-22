import pychrono
import pychrono.core as pc
import pychrono.visual as v
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- Simulation Parameters ---
terrain_height = 10.0  # Meters
friction = 0.8  # Coefficient of friction
restitution = 0.7  # Coefficient of restitution
initial_velocity = 10.0  # m/s
acceleration = 2.0  # m/s^2
time_step = 0.1  # seconds
simulation_duration = 60.0  # seconds
num_timesteps = 100

# --- Terrain Setup ---
terrain = pc.Terrain(
    height=terrain_height,
    friction=friction,
    restitution=restitution,
    mass=10.0,  # kg
    surface_type="grass"  # Can be "grass", "dirt", "stone"
)

# --- Vehicle Setup ---
vehicle = pc.Vehicle(
    mass=10.0,
    initial_velocity=initial_velocity,
    acceleration=acceleration,
    position=[0.0, 0.0, 0.0],  # Initial position (x, y, z)
    orientation=[0.0, 0.0, 0.0]  # Initial orientation (roll, pitch, yaw)
)

# --- Driver System ---
driver = pc.Driver(
    mass=10.0,
    acceleration=acceleration,
    position=[0.0, 0.0, 0.0],
    orientation=[0.0, 0.0, 0.0]
)


# --- Visualization ---
fig, ax = v.figure(size=(800, 600))
scene = v.Scene()
camera = v.Camera(
    position=[0, 0, 0],
    aspect=2.0,
    focal_length=50.0,
    interpolation=v.Interpolation.BILINEAR
)

# Initialize the camera with the scene
scene.add(camera)

# --- Simulation Loop ---
for t in range(num_timesteps):
    # 1. Update Vehicle
    vehicle.accelerate(acceleration * time_step)
    vehicle.update_position(vehicle.position)

    # 2. Update Terrain
    terrain.update()

    # 3. Update Driver
    driver.accelerate(0) # Driver doesn't need to move much

    # 4. Update Visualization
    scene.update()

    # 5. Draw the Scene
    v.draw_scene(scene)

    # 6. Display the Scene
    v.display(scene)

    # 7.  Print Timestep
    print(f"Time Step: {t}")

# --- End of Simulation ---
#  You can add more sophisticated visualization or analysis here.
#  For example, you could save the animation to a file.
#  For example:
#  v.save_animation("kraz_simulation.mp4")