import pychrono
import pychrono.core as pc
import pychrono.visual as pv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


try:
    chrono = pc.chrono.Chrono()
except Exception as e:
    print(f"Error initializing Chrono: {e}")
    exit()



rotor_model = pc.Model("JeffcottRotor")
rotor_model.set_mass(1000)  
rotor_model.set_length(1.0)  
rotor_model.set_rotation_speed(0.1) 


flywheel_model = pc.Model("Flywheel")
flywheel_model.set_mass(1000)  
flywheel_model.set_length(0.5)  
flywheel_model.set_rotation_speed(0.05) 


simulation_time = 10  
time_step = 0.01  
num_steps = int(simulation_time / time_step)


beam = pc.Beam(
    model=rotor_model,
    length=1.0,  
    mass=1000, 
    rotation_speed=rotor_model.set_rotation_speed(),
    
    support_radius=0.1,
    support_height=0.5
)


flywheel_position = 0.0  
flywheel_location = beam.get_position()
flywheel_model.set_position(flywheel_location + flywheel_position)


motor = pc.Motor(
    model=pc.Motor("Motor"),
    speed=rotor_model.set_rotation_speed(),
    
    damping=0.1
)


for i in range(num_steps):
    
    motor.set_position(motor.get_position() + motor.get_speed() * time_step)

    
    rotor_model.set_force(flywheel_model, 0.0)  
    motor.set_force(0.0, 0.0)  

    
    beam.set_position(beam.get_position() + time_step)

    
    pv.plot(beam, title="Jeffcott Rotor Beam Dynamics")
    pv.show(beam)

    
    print(f"Step {i+1}: Time = {time_step:.2f} s")


chrono.close()