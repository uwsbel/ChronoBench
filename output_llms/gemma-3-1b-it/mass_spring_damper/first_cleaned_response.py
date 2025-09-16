import pychrono
import pychrono.core as pc
import pychrono.visual as v
import numpy as np
import matplotlib.pyplot as plt
from pychrono.visual import Visual

def simulate_mass_spring_damper(dt=0.01):
    

    
    chrono = pc.Chchrono()
    chrono.set_environment(pc.Environment(
        time_step=dt,
        gravity=0.0,  
        mass=1.0,
        spring_constant=0.5,
        damping_coefficient=0.1
    ))

    
    mass = pc.Mass(0.1, 1.0)  
    spring = pc.Spring(0.2, 0.5)  
    ground = pc.Ground(0.0, 1.0)  

    
    chrono.set_default_parameters(
        mass=mass,
        spring_constant=spring.constant,
        damping_coefficient=damping_coefficient
    )

    
    visual = v.Visual(
        title="Mass-Spring-Damper Simulation",
        camera_settings={"width": 640, "height": 480},
        
        lighting={"color": "white"}
    )

    
    chrono.create_link(mass, ground)

    
    chrono.run(visual)

    
    

if __name__ == '__main__':
    simulate_mass_spring_damper()