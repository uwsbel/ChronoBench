import pychrono
import pychrono.core as pc
import pychrono.visual as v
import numpy as np
from pychrono.visual.camera import Camera
from pychrono.visual.screen import Screen
from pychrono.visual.ui import UI
import random


simulation_time = 100  
time_step = 0.1  
initial_position = [0, 0]  
initial_velocity = 0  
gravity = 9.81  
terrain_heightmap = np.zeros((100, 100))  
vehicle_mass = 1000  
driver_acceleration = 1.0  
driver_steering_rate = 0.1  
inertial_measurement_unit_data = np.zeros((1, 1))  
gps_data = np.zeros((1, 1))  
camera_width = 600
camera_height = 400
camera_aspect = 0.5
camera_zoom = 1.0


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
        
    )
)


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


def update_simulation():
    global time
    global vehicle_mass
    global driver_acceleration
    global driver_steering_rate

    
    imu.update()
    gps.update()

    
    vehicle_mass_change = vehicle_mass * time_step
    vehicle_mass = vehicle_mass - driver_acceleration * time_step

    
    terrain_shift = random.uniform(-0.1, 0.1)
    terrain_heightmap[0] += terrain_shift
    terrain_heightmap[-1] -= terrain_shift

    
    driver_steering_rate += 0.01  
    driver_steering_rate = max(0.0, min(driver_steering_rate, 1.0))  

    
    screen.update()
    screen.render(terrain_heightmap)
    screen.render(vehicle_mass)
    screen.render(driver_acceleration)

    
    time += time_step

    
    print(f"Time: {time:.2f} s")
    print(f"Vehicle Mass: {vehicle_mass}")
    print(f"Driver Acceleration: {driver_acceleration}")
    print(f"Driver Steering Rate: {driver_steering_rate}")


update_simulation()


simulation.end()