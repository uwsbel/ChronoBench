import pychrono
import pychrono.core as pc
import pychrono.objects as o
import pychrono.visuals as v
import numpy as np


gravity = 9.81  
length = 1.0  
mass = 1.0  
damping = 0.1  
initial_angle = np.pi / 4  
initial_velocity = 0.0  


pendulum = o.Pendulum(mass, length, initial_angle, damping)


ground_body = o.GroundBody(length)


joint = o.RevoluteJoint(pendulum.length, 0.0, 0.0, 0.0)  


v.Render(pendulum, ground_body, joint,  
          title="Simple Pendulum Simulation",
          width=600, height=400,  
          camera_position=(0, 0, 0),
          camera_look_at=(0, 0, 0),
          fps=30)  


chrono.init()


time = 0.0
while time < 10.0:  
    
    pendulum.update_position(time)
    pendulum.update_velocity(time)

    
    chrono.log(pendulum, time)

    
    print(f"Time: {time:.2f}")

    time += 0.1  

chrono.cleanup()