import pychrono as pychrono
from pychrono import Vehicle, Driver, RigidBody, Physics
from pychrono.trajectory import Trajectory
from pychrono.utils import *
from pychrono.render import *
import time

# Initialize PyChrono
pychrono.init()

# Set up physics and visualization
physics = Physics()
physics.set_real_time(True)
physics.set_timestep(1.0/60)  # 60 FPS

# Create vehicle
vehicle = Vehicle("M113", mass=3000,  # Mass in kg
                  dimensions=[2.5, 2.5, 1.2],  # [length, width, height] in meters
                  inertia=[[4.0, 0, 0], [0, 4.0, 0], [0, 0, 4.0]])  # Inertia matrix in kg m^2
vehicle.set_initial_position([0, 0, 0])
vehicle.set_initial_orientation(quat_from_euler([0, 0, 0, 1]))
vehicle.set_initial_velocity([0, 0, 0])

# Create driver
driver = Driver("Driver")
driver.attach_to(vehicle)

# Create terrain
terrain = RigidBody("Terrain")
terrain.set_mass(0)
terrain.set_friction(0.5)  # Friction coefficient
terrain.set_restitution(0.3)  # Restitution coefficient
terrain.create_plane(position=[0, 0, 0], size=[100, 100, 1])
terrain.set_ground(True)

# Visualization setup
renderer = create_renderer()
renderer.set_camera_distance(5, 5, 5)  # Camera position
renderer.set_light_setup()
renderer.set_light_direction(10, 10, 10)  # Lighting

# Create simulation loop
trajectory = Trajectory()
trajectory.set_name("M113_Simulation")

while True:
    physics.update()
    time.sleep(1.0/60)  # Wait for physics update
    
    # Get states
    vehicle_state = vehicle.get_state()
    terrain_state = terrain.get_state()
    
    # Get driver input
    speed = vehicle_state.velocity().z
    yaw = vehicle_state.yaw
    
    # Handle user input for controls
    keys = pygame.keys()
    speed = 0
    steering = 0
    
    if 'W' in keys:
        speed = 1
    elif 'S' in keys:
        speed = -1
    if 'A' in keys:
        steering = 0.5
    elif 'D' in keys:
        steering = -0.5
    
    # Apply controls
    driver.set_throttle(speed)
    driver.set_steering(steering)
    
    # Update visualization
    renderer.update()
    
    # Record trajectory
    trajectory.add_point(vehicle_state.position)
    trajectory.add_vector(vehicle_state.velocity)
    
    # Display info
    print(f"Speed: {speed:.2f} m/s | Yaw: {yaw:.2f} rad")
    
    # Close window on 'Q' key
    for event in pygame.event.get():
        if event.type == pygame.QUIT and 'Q' in keys:
            pygame.quit()
            exit()