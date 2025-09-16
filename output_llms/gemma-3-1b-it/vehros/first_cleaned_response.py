import pychrono
import numpy as np
import time
import random


simulation_time = 100  
dt = 0.1  
terrain_friction = 0.9  
terrain_restitution = 0.8  
vehicle_type = "SUV"  
contact_method = "Contact"  
engine_type = "Diesel"
tire_model = "All-Terrain"
initial_position = [0, 0, 0]
initial_velocity = 10  
initial_acceleration = 0.1  
initial_steering_angle = 0
initial_throttle = 0
initial_brake = 0
initial_radius = 1.0 


pychrono.init()



vehicle = pychrono.Vehicle(
    id="vehicle",
    type=vehicle_type,
    position=initial_position,
    velocity=initial_velocity,
    acceleration=initial_acceleration,
    steering_angle=initial_steering_angle,
    throttle=initial_throttle,
    brake=initial_brake,
    radius=initial_radius
)


terrain = pychrono.Terrain(
    id="terrain",
    type=terrain_friction,
    friction=terrain_friction,
    restitution=terrain_restitution,
    heightmap=np.random.rand(100, 100)  
)


ros_manager = pychrono.Manager()
ros_manager.register_handlers(
    "vehicle_handler",
    lambda event: print(f"Vehicle Event: {event}")
)


try:
    while True:
        
        
        vehicle.update(dt)

        
        terrain.update(dt)

        
        
        
        driver_input = {
            "throttle": initial_throttle,
            "brake": initial_brake,
            "steering_angle": initial_steering_angle
        }
        print(f"Driver Input: {driver_input}")

        
        ros_manager.publish(
            "vehicle_state",
            {
                "position": vehicle.position,
                "velocity": vehicle.velocity,
                "acceleration": vehicle.acceleration,
                "steering_angle": vehicle.steering_angle,
                "throttle": vehicle.throttle,
                "brake": vehicle.brake
            }
        )

        
        time.sleep(dt)

except KeyboardInterrupt:
    print("Simulation interrupted.")
    print("ROS Manager:  Data published successfully.")