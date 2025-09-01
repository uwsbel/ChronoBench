import pychrono
import pychrono.core as chrono
import pychrono.visuals as visuals
import pychrono.events as events
import numpy as np


gravity = 9.81  
initial_angle = 30.0  
initial_angular_velocity = 0.0
initial_position = 0.0
initial_velocity = 0.0
simulation_duration = 10.0  
num_steps = 100


mass = 1.0  
length = 1.0  
joint_length = 0.5  

ground_body_position = 0.0
ground_body_velocity = 0.0


pendulum_mass = mass
pendulum_inertia = 1.0  
pendulum_angle = initial_angle
pendulum_angular_velocity = initial_angular_velocity


joint_angle = 0.0
joint_radius = 0.05 
joint_length = 0.5



pendulum_position = pendulum_position
pendulum_velocity = pendulum_angular_velocity


for i in range(simulation_duration):
    
    pendulum_position += pendulum_angular_velocity * i
    pendulum_velocity += pendulum_angular_velocity * i

    
    
    
    
    force = 0.0  
    force = force * (pendulum_mass / length) 
    force = force * (pendulum_inertia / length) 
    force = force * (1.0 - np.cos(pendulum_angle)) 
    pendulum_position += force * (pendulum_mass / length)

    
    joint_angle += joint_angle * 0.1  

    
    visuals.plot(pendulum_position, pendulum_velocity,  
                label="Pendulum Position", color="blue")
    visuals.plot(joint_angle, joint_radius, color="red")
    visuals.plot(pendulum_angle, pendulum_inertia, color="green")

    
    if i % 10 == 0:
        print(f"Step {i}: Pendulum Position = {pendulum_position:.2f}, Velocity = {pendulum_velocity:.2f}")

    
    
    
    
    



visuals.plot(pendulum_position, pendulum_velocity,  
                label="Pendulum Position", color="blue")
visuals.plot(joint_angle, joint_radius, color="red")
visuals.plot(pendulum_angle, pendulum_inertia, color="green")
visuals.plot(pendulum_angle, pendulum_mass, color="yellow") 
visuals.plot(pendulum_mass, 0.0, color="black") 
visuals.plot(pendulum_mass, 0.0, color="black") 

visuals.plot(pendulum_angle, 0.0, color="red") 
visuals.plot(pendulum_angle, 0.0, color="red") 


print("Simulation complete.")