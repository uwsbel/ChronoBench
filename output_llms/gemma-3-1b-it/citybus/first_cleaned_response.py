import pychrono
import irlayout
import random
import numpy as np

class CityBusSimulation:
    def __init__(self):
        self.chrono = pychrono.Chrono()
        self.vehicle = None
        self.terrain = None
        self.camera_position = (0, 0, 0)  
        self.camera_rotation = 0  
        self.vehicle_speed = 0.5  
        self.tire_model = "rubber" 
        self.terrain_texture = "grass" 
        self.frame_rate = 50
        self.time = 0

    def initialize_environment(self):
        
        self.chrono.start_time = self.time
        self.chrono.set_time_step(self.frame_rate)
        self.chrono.set_time_step_duration(self.frame_rate)
        self.vehicle = self.create_vehicle()
        self.terrain = self.create_terrain()
        self.camera_position = self.get_camera_position()
        self.camera_rotation = 0
        self.vehicle_speed = self.get_vehicle_speed()

    def create_vehicle(self):
        
        return CityBus(self.vehicle_speed, self.vehicle_model, self.vehicle_location, self.vehicle_orientation, self.vehicle_tire_model, self.terrain_texture)

    def get_camera_position(self):
        
        return self.camera_position

    def get_vehicle_speed(self):
        
        return self.vehicle_speed

    def create_terrain(self):
        
        return self.terrain

    def get_terrain_texture(self):
        
        return self.terrain_texture

    def update_vehicle(self):
        
        
        self.vehicle_speed += 0.1
        self.vehicle_speed = max(0.1, min(self.vehicle_speed, 5.0))  

        
        self.vehicle_orientation = self.vehicle_orientation + 0.05

        
        self.render()

    def render(self):
        
        
        self.mesh = self.create_mesh()
        self.camera_position = self.get_camera_position()
        self.camera_rotation = 0
        self.render_scene(self.mesh, self.camera_position, self.camera_rotation)

    def render_scene(self, mesh, position, rotation):
        
        irlayout.draw_mesh(mesh, position, rotation)

    def run(self):
        
        self.chrono.run_time = self.frame_rate
        self.chrono.set_time_step_duration(self.frame_rate)
        self.chrono.start_time = self.time
        self.chrono.set_time_step_duration_seconds(self.frame_rate)
        self.chrono.set_time_step_interval(self.frame_rate)

        self.update_vehicle()
        self.chrono.wait() 

        self.chrono.stop_time = self.time
        self.chrono.set_time_step_duration_seconds(self.frame_rate)
        self.chrono.set_time_step_interval(self.frame_rate)


if __name__ == "__main__":
    simulation = CityBusSimulation()
    simulation.run()