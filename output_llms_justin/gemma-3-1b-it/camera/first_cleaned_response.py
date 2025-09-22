import pychrono
import numpy as np
import matplotlib.pyplot as plt
from pychrono.core import System
from pychrono.core import Scene
from pychrono.core import Physics
from pychrono.core import Camera
from pychrono.core import SensorManager
from pychrono.core import Simulation


try:
    
    chrono = pychrono.core.Core()
    chrono.initialize()
    print("PyChrono initialized successfully.")
except Exception as e:
    print(f"Error initializing PyChrono: {e}")
    exit()



try:
    mesh = chrono.load_mesh("path/to/your/triangle.obj")
except Exception as e:
    print(f"Error loading mesh: {e}")
    exit()


object_name = "triangle"  
object = chrono.create_object(object_name)


sensor_manager = SensorManager(object)


camera = chrono.create_camera(
    object_name,
    sensor_manager=sensor_manager,
    camera_position=(0, 0, 5),  
    camera_orientation=0,       
    camera_resolution=100,      
    
)



initial_position = (0, 0, 5)
initial_force = 10.0
initial_velocity = 0.0


simulation_time = 10  
time_step = 0.1  
simulation_rate = 10  


try:
    simulation = chrono.create_simulation(
        simulation_time,
        simulation_rate,
        initial_position,
        initial_force,
        initial_velocity,
        object
    )
    print("Simulation started.")

    
    for t in range(simulation_time):
        
        camera.position = camera.position + camera.velocity * time_step
        
        print(f"Time: {t}, Camera Position: {camera.position}")

        
        simulation.update()

    print("Simulation finished.")

except Exception as e:
    print(f"Simulation failed: {e}")
    exit()



simulation.stop()
print("Simulation stopped.")



try:
    plt.imshow(camera.image, cmap='gray')
    plt.title("Camera Image")
    plt.show()
except Exception as e:
    print(f"Visualization failed: {e}")