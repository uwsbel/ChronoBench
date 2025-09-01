import pychrono
import numpy as np
import matplotlib.pyplot as plt
from pychrono.environment import System
from pychrono.environment import Terrain
from pychrono.visual import Irrlicht
import time


simulation_name = "HMMWV Simulation"
simulation_time = 10  
initial_position = [0, 0, 0]  
initial_velocity = [0, 0, 0]  
initial_force = [0, 0, 0]  
terrain_heightmap_path = "terrain.png"  
terrain_heightmap_size = 100 


system = System(simulation_name, initial_position, initial_velocity, initial_force)
system.set_physics_system(pychrono.physics_system.Fluid) 
system.set_gravity(0) 
system.set_simulation_time(simulation_time)



terrain = Terrain(
    initial_position,
    terrain_heightmap_path,
    terrain_heightmap_size,
    
    
)


driver = Driver(system, "Driver")
driver.set_speed(initial_velocity) 
driver.set_acceleration(0) 


irrlicht = Irrlicht.initialize()
irrlicht.set_color("blue")
irrlicht.set_width(800)
irrlicht.set_height(600)
irrlicht.set_title(simulation_name)
irrlicht.rotate(np.pi / 2)  


def update_simulation():
    global system, terrain, driver, irrlicht

    
    system.update()
    vehicle_position = system.get_position()
    vehicle_velocity = system.get_velocity()
    vehicle_force = system.get_force()

    
    terrain.update()

    
    driver.update()

    
    irrlicht.clear()
    irrlicht.set_aspect(1)  
    irrlicht.set_camera_position(vehicle_position)
    irrlicht.set_camera_look_at(terrain.get_center())

    
    irrlicht.plot(terrain.get_heightmap())

    
    irrlicht.plot(vehicle_position)
    irrlicht.plot(vehicle_velocity)
    irrlicht.plot(vehicle_force)

    irrlicht.pause(0.01) 



try:
    while True:
        update_simulation()
        time.sleep(0.1)  

except KeyboardInterrupt:
    print("Simulation interrupted.")
    sys.exit()