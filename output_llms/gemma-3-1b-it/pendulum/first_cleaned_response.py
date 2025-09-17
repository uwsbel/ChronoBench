import pychrono
import pychrono.core as pc
import pychrono.visuals as v
import numpy as np
import matplotlib.pyplot as plt


g = 9.81  
L = 1.0  
m = 1.0  
theta_0 = 45.0  
omega_0 = 0.0  
dt = 0.01  
t_end = 10.0  
num_steps = int(t_end / dt)


pendulum = pc.Pendulum(L, m, theta_0, omega_0, dt)


ground_body = pc.GroundBody(L)


joint = pc.RevoluteJoint(pendulum, ground_body)


pendulum.set_state(np.array([0.0, 0.0, 0.0]))  


for i in range(num_steps):
    
    pendulum.set_state(np.array([0.0, 0.0, 0.0]))  

    
    angular_acceleration = -g / L * pendulum.inertia.x

    
    pendulum.set_velocity(pendulum.inertia.x * angular_acceleration)

    
    v.log_state(pendulum, i, t=i)

    
    v.visualize(pendulum, ground_body, joint)
    plt.pause(0.01)  


print("Simulation complete.")