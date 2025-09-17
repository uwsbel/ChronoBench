import pychrono
import time
import random
import math
import irlayout as ir


SIMULATION_TIME = 60  
NUM_STEPS = 100
INITIAL_POSITION = (0, 0, 0)
INITIAL_FORCE = 10.0
INITIAL_IMU_DATA = {
    'acceleration': [0.0, 0.0, 0.0],
    'gyro': [0.0, 0.0, 0.0],
    'angular_velocity': [0.0, 0.0, 0.0]
}
INITIAL_GPS_DATA = {
    'latitude': 34.0522,
    'longitude': -118.2437,
    'altitude': 0.0
}


HMMWV_TYPE = "HMMWV"
HMMWV_SIZE = 5  

TERRAIN_HEIGHT = 1.0
TERRAIN_WIDTH = 0.5


IMU_DATA_RANGE = 0.1  
IMU_DATA_EPS = 0.01 


VISUAL_WIDTH = 600
VISUAL_HEIGHT = 400
VISUAL_COLOR = (255, 255, 255)  


def initialize_chrono():
    
    print("Initializing PyChrono...")
    chrono.init()
    print("PyChrono initialized.")

def setup_vehicle(position, force, initial_imu_data, initial_gps_data):
    
    print("Setting up vehicle...")
    vehicle_mass = 100.0
    vehicle_speed = 0.0
    vehicle_acceleration = 0.0
    vehicle_gyro = 0.0
    vehicle_angular_velocity = 0.0
    
    vehicle_position = position
    vehicle_force = force
    vehicle_imu_data = initial_imu_data
    vehicle_gps_data = initial_gps_data
    
    return vehicle_mass, vehicle_speed, vehicle_acceleration, vehicle_gyro, vehicle_angular_velocity

def update_vehicle(time_step):
    
    global vehicle_mass, vehicle_speed, vehicle_acceleration, vehicle_gyro, vehicle_angular_velocity
    
    
    vehicle_gyro += 0.01  
    vehicle_gyro = max(0.0, vehicle_gyro) 
    
    vehicle_angular_velocity += 0.01 
    vehicle_angular_velocity = max(0.0, vehicle_angular_velocity)
    
    
    vehicle_gps_data['latitude'] = vehicle_position[0]
    vehicle_gps_data['longitude'] = vehicle_position[1]
    vehicle_gps_data['altitude'] = vehicle_position[2]
    
    
    vehicle_force += 0.1
    
    
    vehicle_acceleration = vehicle_force / vehicle_mass
    
    
    vehicle_mass += 0.1
    vehicle_speed += vehicle_acceleration
    vehicle_gyro += 0.01
    vehicle_angular_velocity += 0.01
    
    return vehicle_mass, vehicle_speed, vehicle_acceleration, vehicle_gyro, vehicle_angular_velocity

def render_scene(vehicle_mass, vehicle_speed, vehicle_acceleration, vehicle_gyro, vehicle_angular_velocity):
    
    try:
        ir.board(VISUAL_WIDTH, VISUAL_HEIGHT, VISUAL_COLOR)
        vehicle_mass_label = ir.text(f"{vehicle_mass:.2f}", (VISUAL_WIDTH // 2, VISUAL_HEIGHT // 2))
        vehicle_speed_label = ir.text(f"{vehicle_speed:.2f}", (VISUAL_WIDTH // 2, VISUAL_HEIGHT // 2))
        vehicle_acceleration_label = ir.text(f"{vehicle_acceleration:.2f}", (VISUAL_WIDTH // 2, VISUAL_HEIGHT // 2))
        vehicle_gyro_label = ir.text(f"{vehicle_gyro:.2f}", (VISUAL_WIDTH // 2, VISUAL_HEIGHT // 2))
        vehicle_angular_velocity_label = ir.text(f"{vehicle_angular_velocity:.2f}", (VISUAL_WIDTH // 2, VISUAL_HEIGHT // 2))
    except Exception as e:
        print(f"Error rendering: {e}")

def simulate_step():
    
    global vehicle_mass, vehicle_speed, vehicle_acceleration, vehicle_gyro, vehicle_angular_velocity
    
    
    vehicle_mass, vehicle_speed, vehicle_acceleration, vehicle_gyro, vehicle_angular_velocity = update_vehicle(SIMULATION_TIME / NUM_STEPS)
    
    
    render_scene(vehicle_mass, vehicle_speed, vehicle_acceleration, vehicle_gyro, vehicle_angular_velocity)
    
    
    print(f"Sensor Data: {vehicle_mass:.2f}, {vehicle_speed:.2f}, {vehicle_acceleration:.2f}, {vehicle_gyro:.2f}, {vehicle_angular_velocity:.2f}")
    
    
    time.sleep(1 / NUM_STEPS)


if __name__ == "__main__":
    
    initialize_chrono()

    
    for step in range(NUM_STEPS):
        simulate_step()
    
    print("Simulation complete.")