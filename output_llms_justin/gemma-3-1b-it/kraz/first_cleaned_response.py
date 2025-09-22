import pychrono
import numpy as np
import matplotlib.pyplot as plt
from pychrono.objects import Vehicle, Terrain, Driver, Camera, Light
from pychrono.simulation import Simulation


try:
    simulation = pychrono.simulation.Simulation()
    print("PyChrono simulation initialized.")
except Exception as e:
    print(f"Error initializing PyChrono: {e}")
    exit()




vehicle = Vehicle()
vehicle.name = "Kraz"
vehicle.position = np.array([0.0, 0.0, 0.0])  
vehicle.mass = 100.0
vehicle.friction = 0.8
vehicle.restitution = 0.7
vehicle.initial_velocity = 0.0
vehicle.rotation_rate = 0.0


terrain = Terrain()
terrain.name = "Rocky Terrain"
terrain.friction = 0.9
terrain.restitution = 0.8
terrain.position = np.array([0.0, 0.0, 0.0])  
terrain.initial_velocity = 0.0


driver = Driver()
driver.name = "Driver"
driver.position = np.array([0.0, 0.0, 0.0])
driver.mass = 50.0
driver.rotation_rate = 0.0
driver.acceleration = np.array([0.0, 0.0, 0.0])


simulation.set_default_parameters(
    vehicle_mass=100.0,
    vehicle_friction=0.8,
    vehicle_restitution=0.7,
    terrain_friction=0.9,
    terrain_restitution=0.8,
    driver_mass=50.0,
    driver_acceleration=0.0,
    simulation_time=10.0  
)



camera = Camera()
camera.name = "Irrlicht Camera"
camera.resolution = 800  
camera.rotation_angle = 0.0
camera.lighting = Light()
camera.lighting.color = (1.0, 1.0, 1.0)
camera.lighting.intensity = 0.5
camera.camera_settings = {
    "camera_resolution": camera.resolution,
    "camera_rotation": camera.rotation_angle,
    "camera_lighting": camera.lighting
}



try:
    simulation.run(
        vehicle,
        terrain,
        driver,
        camera
    )
except Exception as e:
    print(f"Simulation failed: {e}")
    exit()









print("Simulation complete.")