import pychrono
import pychrono.core as pc
import pychrono.visual as pv
import numpy as np
import matplotlib.pyplot as plt
from pychrono.visual.mesh import Mesh
from pychrono.visual.rigid_terrain import RigidTerrain
from pychrono.visual.texture import Texture
import time


simulation_duration = 60  
frames_per_second = 50
time_step = 0.01
gravity = 9.81


try:
    chrono = pc.chrono.Chrono(
        simulation_duration=simulation_duration,
        frames_per_second=frames_per_second,
        time_step=time_step,
        gravity=gravity,
        display_mode='interactive' 
    )
except Exception as e:
    print(f"Error initializing Chrono: {e}")
    exit()



vehicle = pc.Vehicle(
    position=(0, 0, 0),  
    orientation=(0, 0, 0),  
    contact_method='tire_model', 
    tme_asyi_tire_model='rigid_terrain_model', 
    
)


terrain = RigidTerrain(
    width=10,  
    height=5,  
    texture='terrain.png',  
    
)



mesh = Mesh(vehicle)


rigid_terrain = RigidTerrain(
    width=10,
    height=5,
    texture='terrain.png',
    
)


try:
    while True:
        
        vehicle.update_dynamics()
        vehicle.update_orientation() 

        
        terrain.update()

        
        pv.render(mesh, terrain,  
                   fov=60,  
                   num_buffers=1)

        
        print(f"Frame: {time.time()}")

        
        vehicle.steer(0, 0, 0) 

        time.sleep(0.01)  

except KeyboardInterrupt:
    print("Simulation stopped.")