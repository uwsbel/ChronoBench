import pychrono
import pychrono.core as pc
import pychrono.visual as v
import numpy as np
from pychrono.visual.camera import Camera
from pychrono.visual.screen import Screen
from pychrono.visual.ui import UI
import random

# Simulation Parameters
simulation_time = 100  # seconds
time_step = 0.1  # seconds
initial_position = [0, 0]  # meters
initial_velocity = 0  # m/s
gravity = 9.81  # m/s^2
terrain_heightmap = np.zeros((100, 100))  # meters -  heightmap for terrain
vehicle_mass = 1000  # kg
driver_acceleration = 1.0  # m/s^2
driver_steering_rate = 0.1  # rad/s
inertial_measurement_unit_data = np.zeros((1, 1))  # Raw IMU data
gps_data = np.zeros((1, 1))  # Raw GPS data
camera_width = 600
camera_height = 400
camera_aspect = 0.5
camera_zoom = 1.0

# Initialize PyChrono Environment
simulation = pc.Simulation(
    name="HMMWV_Simulation",
    time_step=time_step,
    initial_position=initial_position,
    initial_velocity=initial_velocity,
    gravity=gravity,
    terrain_heightmap=terrain_heightmap,
    vehicle_mass=vehicle_mass,
    driver_acceleration=driver_acceleration,
    driver_steering_rate=driver_steering_rate,
    inertial_measurement_unit_data=inertial_measurement_unit_data,
    gps_data=gps_data,
    camera=Camera(width=camera_width, height=camera_height, aspect=camera_aspect, zoom=camera_zoom),
    ui=UI(
        screen=Screen(width=camera_width, height=camera_height, aspect=camera_aspect),
        title="HMMWV Simulation",
        # Add UI elements here as needed (e.g., buttons, sliders)
    )
)

# Initialize IMU and GPS Sensors
imu = pc.IMU(
    name="IMU",
    initial_acceleration=0.0,
    initial_gyro=0.0,
    initial_angular_velocity=0.0
)
gps = pc.GPS(
    name="GPS",
    initial_latitude=0.0,
    initial_longitude=0.0
)

# Simulation Loop
def update_simulation():
    global time
    global vehicle_mass
    global driver_acceleration
    global driver_steering_rate

    # 1. Sensor Data Update
    imu.update()
    gps.update()

    # 2. Vehicle Dynamics (Simplified)
    vehicle_mass_change = vehicle_mass * time_step
    vehicle_mass = vehicle_mass - driver_acceleration * time_step

    # 3. Terrain Update (Simple - just a shift)
    terrain_shift = random.uniform(-0.1, 0.1)
    terrain_heightmap[0] += terrain_shift
    terrain_heightmap[-1] -= terrain_shift

    # 4. Driver Input (Simplified)
    driver_steering_rate += 0.01  # Adjust for speed
    driver_steering_rate = max(0.0, min(driver_steering_rate, 1.0))  # Clamp steering

    # 5. Visualization (Basic)
    screen.update()
    screen.render(terrain_heightmap)
    screen.render(vehicle_mass)
    screen.render(driver_acceleration)

    # 6.  Time Step Update
    time += time_step

    # 7.  Print Simulation Information (for debugging)
    print(f"Time: {time:.2f} s")
    print(f"Vehicle Mass: {vehicle_mass}")
    print(f"Driver Acceleration: {driver_acceleration}")
    print(f"Driver Steering Rate: {driver_steering_rate}")

# Run the Simulation
update_simulation()

# Simulation End
simulation.end()