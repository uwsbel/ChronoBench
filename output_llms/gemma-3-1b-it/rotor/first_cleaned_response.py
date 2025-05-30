import pychrono
import pychrono.physics as pc
import pychrono.visual as cv
import numpy as np
import matplotlib.pyplot as plt
from pychrono.simulation import Simulation



domain = pc.Domain(
    dimensions=(10, 10, 10),  
    
    noise_level=0.1,
    
    initial_position=(0, 0, 0),
    initial_velocity=(1, 0, 0)
)



rotor_model = pc.Model(
    name="JeffcottRotor",
    
    geometry=pc.Beam(
        length=1.0,
        width=0.5,
        height=0.5,
        
        rotation_angle=np.pi/2, 
        
        damping=0.1
    ),
    
    mass=1.0,
    inertia=1.0,
    
    rotation_speed=10.0,
    
    damping_coefficient=0.05
)


flywheel_model = pc.Model(
    name="Flywheel",
    
    geometry=pc.Beam(
        length=1.0,
        width=0.5,
        height=0.5,
        
        rotation_angle=np.pi/2,
        
        mass=1.0,
        
        inertia=1.0,
        
        rotation_speed=10.0,
        
        damping_coefficient=0.05
    ),
    
    position=(0, 0, 0)
)



simulation = pc.Simulation(
    domain=domain,
    physics_system=pc.System(rotor_model, flywheel_model),
    
    initial_position=domain.initial_position,
    initial_velocity=domain.initial_velocity,
    
    simulation_parameters={"timestep": 0.01}
)



cv.create_visualization(
    domain=domain,
    simulation=simulation,
    
    title="Jeffcott Rotor Simulation",
    
    window_width=600,
    window_height=400,
    
    background_color=(0, 0, 0),
    
    grid_size=10,
    
    color_bar_color=(1, 1, 1)
)




simulation.run()



print(simulation.get_parameters())





cv.show_simulation()