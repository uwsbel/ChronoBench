import pychrono
import pychrono.core as chrono
import pychrono.visuals as visual
import pyrr
import time
import random

# --- Simulation Parameters ---
gravity = 9.81  # m/s^2
ground_body_mass = 1000  # kg (approximate rover mass)
ground_body_radius = 1.0  # m (approximate rover radius)
rover_mass = 150  # kg (approximate rover mass - adjust as needed)
rover_speed = 0.5  # m/s (adjust for rover speed)
simulation_duration = 60  # seconds
time_step = 0.1  # seconds
simulation_rate = 10  # frames per second

# --- Initial Setup ---
chrono.init()

# --- Physical Systems ---
# Define the terrain (simple rectangular grid)
terrain_width = 10
terrain_height = 10
terrain_x = 0
terrain_y = 0
terrain_z = 0

# Create the terrain
terrain = chrono.Terrain(
    x=terrain_x,
    y=terrain_y,
    z=terrain_z,
    width=terrain_width,
    height=terrain_height,
    density=0.5,  # Adjust density for terrain roughness
    # Add some simple obstacles for visual interest
    # (can be replaced with more complex terrain generation)
    # obstacle_x = 2, obstacle_y = 2, obstacle_z = 0
)

# --- Rover Setup ---
rover = chrono.Rover(
    mass=rover_mass,
    position= pyrr.Vector(terrain_x, terrain_y, terrain_z),
    speed=rover_speed,
    orientation=chrono.Orientation(angle=0,  # Default orientation - adjust as needed
                             axis= pyrr.Vector(0, 0, 0))
)

# --- Camera Settings ---
camera_resolution = 800
camera_fov = 60  # degrees
camera_aspect_ratio = 1.0  # Fixed aspect ratio
camera_z_offset = 0.0  # Z-offset for camera position

# --- Visualization ---
visual.init(camera_resolution=camera_resolution,
            camera_aspect_ratio=camera_aspect_ratio,
            camera_z_offset=camera_z_offset)

# --- Simulation Loop ---
for i in range(simulation_rate):
    # 1. Update Rover Position
    rover.move(i * time_step)

    # 2. Simulate Ground Body Movement (Simplified - just a slight shift)
    ground_body_position = rover.position.copy()
    ground_body_position.x += random.uniform(-0.1, 0.1)  # Small random displacement
    ground_body_position.y += random.uniform(-0.1, 0.1)
    ground_body_position.z += random.uniform(-0.1, 0.1)

    # 3.  Simple Force Simulation (Rover moving towards ground)
    # This is a very basic simulation - a more robust approach would involve
    # a more complex force calculation.
    force_magnitude = 0.1
    force_direction = ground_body_position.z - rover.position.z
    
    # Apply force to the rover
    rover.apply_force(force_magnitude * force_direction * time_step)


    # 4.  Rendering
    visual.render(terrain, rover, camera)

    # 5.  Display Time
    time.sleep(time_step)

print("Simulation complete.")