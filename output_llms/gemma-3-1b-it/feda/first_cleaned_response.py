import pychrono
import pychrono.core as pc
import pychrono.visuals as v
import pychrono.physics as ph
import pychrono.ui as ui
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import pyplot as plt


vehicle_name = "FEDA"
terrain_type = "grass"
vehicle_speed = 1.0
simulation_time = 60  
camera_x = 0.0
camera_y = 0.0
camera_z = 0.0
vehicle_position = (0.0, 0.0, 0.0)
vehicle_orientation = (0.0, 0.0, 0.0)
contact_method = "soft" 
tire_model = "standard"

try:
    
    pychrono.init()
    print("PyChrono initialized successfully.")

    
    vehicle = pc.Vehicle(vehicle_name, vehicle_position, vehicle_orientation, contact_method, tire_model)
    print(f"Vehicle '{vehicle_name}' created.")

    
    terrain = v.Terrain(terrain_type, vehicle_position, vehicle_orientation, camera_x, camera_y, camera_z)
    print("Terrain created.")

    
    camera = v.Camera(camera_x, camera_y, camera_z,  camera_z)
    camera.set_position(vehicle_position)
    print("Camera set to vehicle position.")

    
    rigid_terrain = v.RigidTerrain(terrain)
    print("RigidTerrain created.")

    
    for frame in range(simulation_time):
        
        vehicle.update_dynamics()

        
        camera.update_position()

        
        v.render(vehicle, camera, rigid_terrain)

        
        ui.display(vehicle)
        ui.display(camera)
        ui.display(rigid_terrain)

        
        ui.sleep(50)

except Exception as e:
    print(f"An error occurred: {e}")
    print("Please check your simulation parameters and ensure PyChrono is working correctly.")
    exit()