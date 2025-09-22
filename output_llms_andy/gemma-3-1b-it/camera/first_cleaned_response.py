import pychrono
import numpy as np
import matplotlib.pyplot as plt
from pychrono.core import System
from pychrono.core.sensor import Sensor
from pychrono.core.physics import Physics
from pychrono.core.simulation import Simulation
from pychrono.core.visualization import Visualization

def simulate_orbit(mesh_path, camera_sensor_name, simulation_duration, initial_position, initial_velocity, num_steps):
    

    
    system = System(
        name=camera_sensor_name,
        physics=Physics.Physics(
            gravity=0.0,
            mass=1.0,  
            dt=1.0  
        )
    )

    
    try:
        mesh = np.load(mesh_path)
    except FileNotFoundError:
        print(f"Error: Mesh file not found at {mesh_path}")
        return
    except Exception as e:
        print(f"Error loading mesh: {e}")
        return

    
    position = mesh  
    velocity = initial_velocity
    
    
    sensor_manager = Sensor(
        name=camera_sensor_name,
        noise_filter=NoiseFilter(
            sigma=0.1,
            min_sigma=0.1,
            max_sigma=1.0
        ),
        visualization=Visualization.Visualization(
            camera_resolution=100,
            camera_fov=60,
            camera_motion=True,
            
        )
    )

    
    for step in range(num_steps):
        
        system.update(position, velocity)

        
        visualization.render(system, step)

        
        print(f"Step {step+1}: Camera Buffer Data")
        print(system.camera_buffer)

        
        

    print("Simulation complete.")


if __name__ == "__main__":
    
    mesh_file = "triangle.obj"  
    camera_sensor_name = "Camera_Sensor"
    simulation_duration = 60.0  
    initial_position = (0.0, 0.0, 0.0)
    initial_velocity = (1.0, 0.0, 0.0)
    num_steps = 100

    simulate_orbit(mesh_file, camera_sensor_name, simulation_duration, initial_position, initial_velocity, num_steps)