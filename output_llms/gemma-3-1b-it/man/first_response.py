import pychrono
import numpy as np
from pychrono.core import System, Object
from pychrono.visuals import Irrlicht
import matplotlib.pyplot as plt
from matplotlib.colors import Listed

# --- Simulation Parameters ---
TERRAIN_WIDTH = 100
TERRAIN_HEIGHT = 50
TERRAIN_SCALE = 10  # Adjust for terrain size
TERRAIN_TEXTURE_NAME = "terrain.png" # Replace with your terrain image path
DRIVER_SPEED = 5  # m/s
DRIVER_BRAKING_FORCE = 10  # N
DRIVER_STEERING_FORCE = 2  # N
INITIAL_POSITION = [0, 0, 0]  # Initial truck position (x, y, z)
INITIAL_SPEED = 0  # Initial speed (m/s)
INITIAL_BRAKING_FORCE = 0  # Initial braking force (N)
INITIAL_STEERING_FORCE = 0  # Initial steering force (N)
INITIAL_TIME = 0.0

# --- System Setup ---
system = System(
    name="truck_simulation",
    time_step=0.1,
    time_limit=10.0,
    physics_system=True,
    visual_system=True,
    visual_settings=True,
    visualization_settings=True,
    # Add more visual settings as needed, e.g., camera, lighting
)

# --- Physical Systems ---
vehicle = Object(
    name="truck",
    model="man10t",
    position=[INITIAL_POSITION[0], INITIAL_POSITION[1], INITIAL_POSITION[2]],
    speed=INITIAL_SPEED,
    steering=INITIAL_STEERING_FORCE,
    braking=INITIAL_BRAKING_FORCE,
    # Add other vehicle properties as needed (e.g., mass, friction)
)

# --- Tire Model ---
tmeier = TMEASY(
    name="tmeier",
    model="tmeier",
    # Add more tire parameters as needed (e.g., rolling resistance, friction)
    # Example:
    #  rolling_resistance = 0.01,
    #  friction = 0.9,
)

# --- Terrain ---
terrain = np.zeros((TERRAIN_WIDTH, TERRAIN_HEIGHT))
terrain[TERRAIN_WIDTH // 2:TERRAIN_WIDTH // 2 + TERRAIN_HEIGHT, :] = 1.0  # Create a simple terrain

# --- Visualization ---
irrlicht = Irrlicht()
irrlicht.initialize_figure()
irrlicht.rotate_figure(np.pi / 2) # Rotate the view
irrlicht.set_camera(Irrlicht.Camera(position=[0, 0, 0]))
irrlicht.set_background_color(np.array([0.2, 0.2, 0.2])) # Dark gray background

# --- Camera Setup ---
camera = Irrlicht.Camera(position=[0, 0, 0])
camera.set_color(np.array([0.5, 0.5, 0.5]))
camera.set_rotation(np.pi / 2)

# --- Chase Camera ---
camera.set_camera_type(Irrlicht.ChaseCamera)
camera.set_color(np.array([0.7, 0.7, 0.7]))
camera.set_position([0, 0, 0])
camera.set_rotation(np.pi / 2)

# --- Main Simulation Loop ---
while True:
    # --- Update ---
    system.update()

    # --- Visualize ---
    irrlicht.clear()
    irrlicht.set_camera(camera)
    irrlicht.set_background_color(np.array([0.2, 0.2, 0.2]))
    irrlicht.draw_voronoi(vehicle.position)
    irrlicht.draw_polygon(vehicle.position)

    # --- Render ---
    irrlicht.display()

    # --- Wait ---
    import time
    time.sleep(1/system.time_step)