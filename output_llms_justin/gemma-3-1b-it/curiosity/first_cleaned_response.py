import pychrono
import numpy as np
import matplotlib.pyplot as plt
from pychrono.core import System
from pychrono.core.physics import Force
from pychrono.core.visual import Camera
from pychrono.core.visual.visual_elements import Texture
from pychrono.core.visual.lighting import Light


simulation_time = 100  
terrain_height = 5  
rover_mass = 100  
rover_speed = 0.5  
motor_torque = 10  
simulation_start_time = 0
simulation_end_time = simulation_time


system = System(
    name="CuriosityRover",
    initial_position=[0, 0, terrain_height/2],
    initial_velocity=[0, 0, 0],
    initial_torque=motor_torque,
    physics_model="rigid",
    gravity=0,
    
)



ground = Force(
    system,
    name="Ground",
    type="rigid",
    mass=1000,  
    collision_properties=True,
    
)


rover = Force(
    system,
    name="Rover",
    type="rigid",
    mass=rover_mass,
    collision_properties=True,
    
)


camera = Camera(
    system,
    name="Camera",
    resolution=[800, 600],
    
)


light = Light(
    system,
    name="Light",
    color=[0.5, 0.5, 0.5],  
    intensity=1.0,
    
)


texture = Texture(
    system,
    name="Texture",
    path="rover_texture.png", 
    
)



rover.position = system.initial_position
rover.velocity = system.initial_velocity


for time in range(simulation_start_time, simulation_end_time + 1):
    
    rover.update_physics()
    
    
    camera.update_scene()
    
    
    light.update_scene()

    
    system.render()

    
    print(f"Time: {time:.2f}s")


system.end()