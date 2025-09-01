import pychrono
import pychrono.core as pc
import pychrono.visual as v
import numpy as np
import time



terrain_width = 10.0
terrain_height = 5.0
terrain_depth = 2.0


terrain_texture = "terrain.png" 


rigid_terrain = pc.RigidTerrain(
    width=terrain_width,
    height=terrain_height,
    depth=terrain_depth,
    texture=terrain_texture,
    
)



vehicle_speed = 1.0  
vehicle_weight = 100.0  
vehicle_radius = 0.5  
vehicle_center_x = 0.0
vehicle_center_y = 0.0
vehicle_center_z = 0.0
vehicle_orientation = np.array([0.0, 0.0, 0.0])  
vehicle_contact_method = pc.ContactMethod.Primitive.ContactMethod_Point





state_space = [
    vehicle_center_x,
    vehicle_center_y,
    vehicle_center_z,
    vehicle_orientation,
    vehicle_velocity
]


transition_probabilities = {
    "position": [0.5, 0.5, 0.5, 0.5, 0.5],  
    "orientation": [0.7, 0.3, 0.1, 0.1, 0.1],  
    "velocity": [0.2, 0.1, 0.0, 0.0, 0.0] 
}


def run_simulation():
    print("Starting HMMWV Simulation...")
    
    
    pychrono.init(
        initial_time=0.0,
        simulation_time=10.0, 
        initial_state=vehicle_orientation,
        initial_velocity=vehicle_velocity,
        
    )

    
    simulation = pc.Simulation(
        rigid_terrain,
        vehicle_speed,
        vehicle_weight,
        vehicle_radius,
        vehicle_center_x,
        vehicle_center_y,
        vehicle_center_z,
        vehicle_orientation,
        transition_probabilities,
        
    )

    simulation.run()

    print("Simulation completed.")

    
    
    scene = v.Scene(
        width=terrain_width,
        height=terrain_height,
        background=v.Color(0.2, 0.8, 0.2), 
        
    )

    
    vehicle_object = v.Object(
        position=vehicle_center_x,
        orientation=vehicle_orientation,
        size=vehicle_radius,
        color=v.Color(0.5, 0.5, 0.5),  
        
    )
    scene.add(vehicle_object)

    
    v.show(scene)

    
    print("Stopping the vehicle...")
    vehicle_orientation = np.array([0.0, 0.0, 0.0])  
    print("Applying throttle...")
    vehicle_velocity = np.array([0.0, 0.0, 0.0])
    
    
    time.sleep(0.1)
    
    print("Applying throttle...")
    vehicle_velocity = np.array([0.0, 0.0, 0.0])
    
    
    time.sleep(0.1)

    
    print("Simulation finished.")
    pychrono.end(simulation)


if __name__ == "__main__":
    run_simulation()