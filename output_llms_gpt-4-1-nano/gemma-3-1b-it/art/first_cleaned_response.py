import pychrono
import pychrono.core as pc
import pychrono.graphics as pg
import pychrono.utils as pc_utils
import numpy as np
import time




simulation_duration = 60  
frames_per_second = 50
initial_position = [0, 0, 0] 
initial_orientation = [0, 0, 0] 
contact_method = pc_utils.ContactMethod.FORCE_CONTACT  
visualization_type = pc_utils.VisualizationType.3D_VIEW  
vehicle_type = "ARTcar" 
terrain_dimensions = [10, 10, 10] 
texture_name = "artcar_texture" 



terrain = pc.Terrain(dimensions=terrain_dimensions)
terrain.set_texture(texture_name)
terrain.set_gravity(0.0)  


vehicle = pc.Vehicle(
    position=initial_position,
    orientation=initial_orientation,
    contact_method=contact_method,
    vehicle_type=vehicle_type,
    
)



def update_simulation():
    
    global vehicle

    
    vehicle.update_dynamics()

    
    pg.update_scene(vehicle) 

    
    print(f"Time: {time.time()}")
    print(f"Vehicle Position: {vehicle.position}")
    print(f"Vehicle Orientation: {vehicle.orientation}")

    
    pg.update(vehicle) 
    time.sleep(1/frames_per_second) 



if __name__ == "__main__":
    print("Starting ARTcar simulation...")
    simulation = pc.Simulation(
        duration=simulation_duration,
        frames_per_second=frames_per_second,
        initial_position=initial_position,
        initial_orientation=initial_orientation,
        contact_method=contact_method,
        visualization_type=visualization_type,
        vehicle_type=vehicle_type
    )

    simulation.run()
    simulation.wait()  
    print("Simulation finished.")