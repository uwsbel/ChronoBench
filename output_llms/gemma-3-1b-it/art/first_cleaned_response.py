import pychrono
import pychrono.core as chrono
import pychrono.visual as visual
import pychrono.utils as utils
import random
import numpy as np


terrain_width = 10.0
terrain_height = 5.0
terrain_depth = 2.0
vehicle_speed = 0.5
vehicle_acceleration = 0.3
vehicle_deceleration = 0.2
vehicle_steering_sensitivity = 0.1
vehicle_throttle_sensitivity = 0.05
vehicle_brake_sensitivity = 0.02


vehicle_location = (0, 0, 0)
vehicle_orientation = (0, 0, 0)
contact_method = "friction"  
visualization_type = "raytrace" 


terrain = visual.RigidTerrain(
    width=terrain_width,
    height=terrain_height,
    depth=terrain_depth,
    texture="terrain.png",  
    color=(0.8, 0.8, 0.8) 
)


vehicle_mass = 10.0
vehicle_center_of_mass = (vehicle_location[0], vehicle_location[1], vehicle_location[2])
vehicle_radius = 0.5


def run_simulation():
    global vehicle_location, vehicle_orientation, contact_method, visualization_type

    try:
        
        chrono.init()

        
        chrono.set_simulation_parameters(
            time_step=0.01,  
            frame_rate=50,
            gravity=0.0,
            vehicle_mass=vehicle_mass,
            vehicle_center_of_mass=vehicle_center_of_mass,
            vehicle_radius=vehicle_radius,
            vehicle_speed=vehicle_speed,
            vehicle_acceleration=vehicle_acceleration,
            vehicle_deceleration=vehicle_deceleration,
            contact_method=contact_method,
            visualization_type=visualization_type
        )

        
        while True:
            
            vehicle_orientation += vehicle_speed * 0.1  
            vehicle_orientation = np.clip(vehicle_orientation, -1.0, 1.0) 

            
            if contact_method == "friction":
                vehicle_orientation = np.clip(vehicle_orientation, -1.0, 1.0)
                vehicle_orientation = np.clip(vehicle_orientation, -1.0, 1.0)
                vehicle_orientation = np.clip(vehicle_orientation, -1.0, 1.0)
                vehicle_orientation = np.clip(vehicle_orientation, -1.0, 1.0)

            elif contact_method == "elastic":
                vehicle_orientation = np.clip(vehicle_orientation, -1.0, 1.0)
                vehicle_orientation = np.clip(vehicle_orientation, -1.0, 1.0)
                vehicle_orientation = np.clip(vehicle_orientation, -1.0, 1.0)
                vehicle_orientation = np.clip(vehicle_orientation, -1.0, 1.0)

            elif contact_method == "none":
                pass 

            
            visual.update(visualization_type)

            
            visual.display(vehicle_location, vehicle_orientation, vehicle_radius)

            
            time.sleep(0.01)

    except Exception as e:
        print(f"Error: {e}")

    finally:
        
        chrono.cleanup()

if __name__ == "__main__":
    run_simulation()