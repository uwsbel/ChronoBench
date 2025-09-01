import pychrono
import pychrono.core as pc
import pychrono.visuals as v
import pychrono.physics as ph
import pychrono.utils as u
import numpy as np
import matplotlib.pyplot as plt  


terrain_width = 10.0
terrain_height = 5.0
terrain_depth = 2.0
terrain_scale = 10.0
vehicle_width = 2.0
vehicle_length = 3.0
vehicle_mass = 100.0
vehicle_friction = 0.8
vehicle_tire_radius = 0.5
vehicle_tire_pressure = 0.7  
vehicle_tmeasy_model = "tmeasy_model_2" 


chrono = pc.Chronos()


terrain = u.Terrain(
    width=terrain_width,
    height=terrain_height,
    depth=terrain_depth,
    scale=terrain_scale,
    texture="grass.png" 
)


vehicle = pc.Vehicle(
    width=vehicle_width,
    length=vehicle_length,
    mass=vehicle_mass,
    friction=vehicle_friction,
    tmeasy_model=vehicle_tmeasy_model,
    position=(0, 0, 0) 
)



rigid_terrain = pc.RigidTerrain(
    width=terrain_width,
    height=terrain_height,
    depth=terrain_depth,
    scale=terrain_scale,
    texture="grass.png",
    
)



try:
    
    chrono.init(chrono)

    
    while True:
        
        vehicle.update_dynamics()

        
        rigid_terrain.update_terrain()

        
        v.render(vehicle, rigid_terrain,  ) 

        
        print(f"Time: {chrono.time()}")

        
        chrono.wait(1)  

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    
    chrono.cleanup()
    print("Simulation finished.")