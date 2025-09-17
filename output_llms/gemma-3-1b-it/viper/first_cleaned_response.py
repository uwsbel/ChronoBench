import pychrono
import time
import random
import math


simulation_duration = 60  
gravity = 9.81
ground_body_mass = 1000  
ground_body_radius = 10  
rover_mass = 100  
rover_velocity = 0.2  
rover_acceleration = 0.1 
driver_acceleration = 0.05 
steering_rate = 0.01 
camera_width = 640
camera_height = 480
camera_fps = 60
camera_color = (0, 0, 255)  


chrono = pychrono.Chrono()


terrain = pychrono.Terrain()
terrain.set_radius(ground_body_radius)
terrain.set_height(ground_body_radius)
terrain.set_ground_body_mass(ground_body_mass)


rover = pychrono.Rover()
rover.set_mass(rover_mass)
rover.set_velocity(rover_velocity)
rover.set_acceleration(rover_acceleration)
rover.set_driver_acceleration(driver_acceleration)
rover.set_steering_rate(steering_rate)


camera = pychrono.Camera()
camera.set_width(camera_width)
camera.set_height(camera_height)
camera.set_fps(camera_fps)
camera.set_color(camera_color)


try:
    while True:
        
        rover.update()

        
        camera.render()

        
        print(f"Time: {time.localtime().tm_isdst}")
        print(f"Rover Velocity: {rover.velocity}")
        print(f"Rover Acceleration: {rover.acceleration}")
        print(f"Rover Steering: {rover.steering_rate}")
        print(f"Camera Position: {camera.position}")

        
        time.sleep(1)

except KeyboardInterrupt:
    print("Simulation stopped.")
finally:
    
    chrono.stop()
    print("Simulation stopped.")