import time
import numpy as np
from pychrono import Chrono, System
from pychrono.utils import get_system_properties


gravity = 9.81  
timestep = 0.01  
simulation_duration = 10  
max_steps = 100


system = System()
system.set_properties(get_system_properties())  


floor = System.Floor()
floor.set_position(0, 0, 0)  


box = System.Box()
box.set_position(5, 0, 0)  
box.set_mass(10.0)  



floor.set_position(0, 0, 0)


box.set_position(5, 0, 0)


for step in range(simulation_duration):
    
    system.update()

    
    floor.set_position(0, 0, step * gravity)

    
    box.set_position(5, 0, step * gravity)

    
    ros_message = "step_" + str(step)
    print(f"Step {step}: Publishing {ros_message}")
    
    time.sleep(timestep)

print("Simulation complete.")