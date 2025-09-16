import pychrono
import pychrono.core as pc
import pychrono.visual as v
import numpy as np
import matplotlib.pyplot as plt
from pychrono.core import System

class HMMWVSimulation:
    def __init__(self, vehicle_mass, terrain_model, driver_inputs, visual_system):
        
        self.vehicle_mass = vehicle_mass
        self.terrain_model = terrain_model
        self.driver_inputs = driver_inputs
        self.visual_system = visual_system
        self.simulation_time = 10  
        self.time_step = 0.1  
        self.current_time = 0
        self.vehicle_position = np.array([0, 0, 0])  
        self.vehicle_velocity = np.array([0, 0, 0])
        self.sensor_data = {}  
        self.sensor_id = 0  

    def update_sensor_data(self):
        
        for sensor_id, sensor in self.sensor_data.items():
            try:
                
                
                sensor_value = np.random.rand() * 10  
                self.sensor_data[sensor_id] = sensor_value
            except Exception as e:
                print(f"Error updating sensor data for sensor {sensor_id}: {e}")

    def update_vehicle_position(self):
        
        self.vehicle_position = self.vehicle_velocity * self.time_step
        self.vehicle_velocity = self.vehicle_velocity - self.vehicle_mass * self.time_step

    def update_vehicle_velocity(self):
        
        self.vehicle_velocity = self.vehicle_mass * self.time_step

    def run_simulation(self):
        
        self.simulation_time = self.time_step
        while self.current_time < self.simulation_time:
            self.update_sensor_data()
            self.update_vehicle_position()
            self.update_vehicle_velocity()
            self.current_time += self.time_step

        self.visual_system.render(self.vehicle_position, self.vehicle_velocity, self.sensor_data)

    def get_vehicle_mass(self):
        
        return self.vehicle_mass

    def get_sensor_data(self):
        
        return self.sensor_data

    def get_sensor_id(self):
        
        return self.sensor_id

if __name__ == '__main__':
    
    vehicle_mass = 1000.0
    terrain_model = {
        "grass": {"slope": 0.0, "roughness": 0.5},
        "dirt": {"slope": 0.0, "roughness": 0.2},
        "snow": {"slope": 0.0, "roughness": 0.8}
    }
    driver_inputs = {
        "acceleration": 1.0,
        "steering_angle": 0.0
    }
    visual_system = v.Visual(
        title="HMMWV Simulation",
        width=800,
        height=600,
        background="white",
        
    )

    simulation = HMMWVSimulation(vehicle_mass, terrain_model, driver_inputs, visual_system)
    simulation.run_simulation()
    print(f"Vehicle Mass: {simulation.get_vehicle_mass()}")
    print(f"Sensor Data: {simulation.get_sensor_data()}")
    print(f"Sensor ID: {simulation.get_sensor_id()}")