import pychrono
import time
import random
import math


class KrazSimulation:
    def __init__(self, initial_position=0.0, initial_velocity=0.0, friction=0.8, restitution=0.7):
        
        self.position = initial_position
        self.velocity = initial_velocity
        self.friction = friction
        self.restitution = restitution
        self.terrain_friction = 0.9  
        self.terrain_restitution = 0.9 
        self.terrain_radius = 10.0 
        self.terrain_height = 5.0 
        self.time_step = 0.01  
        self.simulation_time = 10.0 
        self.driving_mode = "passive" 
        self.camera_resolution = 800  
        self.camera_position = (0, 0)  
        self.camera_orientation = (math.pi/2, 0, 0) 

    def update(self, timestep):
        
        self.velocity += self.velocity * self.time_step
        self.position += self.velocity * self.time_step
        self.position += self.velocity * timestep * math.sin(self.velocity * timestep)
        self.velocity = max(0.0, min(self.velocity, 10.0)) 
        self.position = (self.position - self.terrain_radius) * self.terrain_restitution 
        self.velocity = max(0.0, min(self.velocity, 10.0)) 


    def get_position(self):
        
        return self.position

    def get_velocity(self):
        
        return self.velocity

    def get_terrain_friction(self):
        
        return self.terrain_friction

    def get_terrain_restitution(self):
        
        return self.terrain_restitution



class Vehicle:
    def __init__(self, position, velocity, friction, restitution):
        
        self.position = position
        self.velocity = velocity
        self.friction = friction
        self.restitution = restitution

    def update(self, timestep):
        
        self.position += self.velocity * timestep
        self.velocity += self.friction * timestep
        self.position = (self.position - self.terrain_radius) * self.terrain_restitution
        self.velocity = max(0.0, min(self.velocity, 10.0))


class Driver:
    def __init__(self, vehicle):
        
        self.vehicle = vehicle

    def update(self, timestep):
        
        
        self.vehicle.update(timestep)



class IrrlichtVisualizer:
    def __init__(self, camera_resolution):
        
        self.camera_resolution = camera_resolution
        self.camera_position = (0, 0)  
        self.camera_orientation = (math.pi/2, 0, 0) 
        self.camera_width = self.camera_resolution
        self.camera_height = self.camera_resolution

    def draw(self, image):
        
        
        self.camera_position = (self.camera_width / 2, self.camera_height / 2)
        self.camera_orientation = (math.pi/2, 0, 0)

        
        self.draw_vehicle(image, self.vehicle.position)

        
        self.draw_terrain(image, self.terrain_radius, self.terrain_height)


    def draw_vehicle(self, image, position):
        
        
        x, y = position
        image.rect(x, y, self.camera_width, self.camera_height)

    def draw_terrain(self, image, radius, height):
        
        
        x, y = position
        image.rect(x, y, radius, height)




if __name__ == "__main__":
    simulation = KrazSimulation(initial_position=0.0, initial_velocity=0.0, friction=0.8, restitution=0.7)
    driver = Driver(simulation)
    visualizer = IrrlichtVisualizer(simulation.camera_resolution)
    visualizer.draw(simulation.camera) 

    print("Simulation started...")
    time_elapsed = 0.0
    while time_elapsed < simulation.simulation_time:
        time_elapsed += 0.01  
        simulation.update(time_elapsed)
        visualizer.draw(simulation.camera) 
        time_elapsed += 0.01