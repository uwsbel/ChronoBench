import pychrono
import numpy as np
import matplotlib.pyplot as plt
import cv2

class GatorSimulation:
    def __init__(self):
        
        self.chrono = pychrono.Chronos()
        self.time = 0
        self.vehicle_position = np.array([0, 0, 0])  
        self.terrain_heightmap = np.zeros((100, 100))  
        self.sensor_manager = self.create_sensor_manager()
        self.camera = self.create_camera()
        self.driver = self.create_driver()

    def create_sensor_manager(self):
        
        sensor_manager = pychrono.SensorManager(
            time_step=1,
            sensor_types=['point_light', 'camera'],
            camera_position=[50, 50, 50],  
            camera_orientation=[0, 0, 0], 
            sensor_type='point_light'
        )
        return sensor_manager

    def create_camera(self):
        
        camera = pychrono.Camera(
            position=[50, 50, 50],
            orientation=[0, 0, 0],
            resolution=[640, 480],
            camera_type='camera'
        )
        return camera

    def create_driver(self):
        
        driver = pychrono.Driver(
            time_step=1,
            speed=1.0,
            acceleration=0.5,
            steering_angle=0.0,
            max_speed=10.0,
            max_acceleration=0.1,
            max_steering_angle=0.0,
            force_factor=1.0,
            force_type='linear'
        )
        return driver

    def update_simulation(self):
        
        self.time += 1
        self.vehicle_position = np.array([self.vehicle_position[0], self.vehicle_position[1], self.vehicle_position[2]])
        self.terrain_heightmap = np.zeros((100, 100))
        self.sensor_manager.update_sensor_data()
        self.camera.update_image()
        self.driver.update_position()

    def visualize_vehicle(self):
        
        
        x = self.vehicle_position[0]
        y = self.vehicle_position[1]
        z = self.vehicle_position[2]

        cv2.line(plt.gca(), self.vehicle_position, self.vehicle_position, (0, 255, 0), 2)  
        cv2.imshow("Vehicle Path", plt.gca())
        plt.show()

    def run_simulation(self):
        
        self.update_simulation()
        self.visualize_vehicle()
        self.chrono.run()

if __name__ == "__main__":
    simulation = GatorSimulation()
    simulation.run_simulation()