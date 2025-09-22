import pychrono
import pychrono.events as events
import pychrono.visuals as visuals
import pychrono.utils as utils
import time
import random


simulation_duration = 60  
initial_position = (0, 0, 0)  
initial_speed = 0.1  
motor_speed = 5.0  
friction_coefficient = 0.8  
joint_strength = 1000  
cylinder_radius = 0.1  
cylinder_height = 0.5 


floor = visuals.Floor(
    x=0,
    y=0,
    z=0,
    radius=cylinder_radius,
    mass=1000,  
    friction=0.1
)


crankshaft = visuals.Crankshaft(
    x=0,
    y=0,
    z=0,
    radius=cylinder_radius,
    mass=1000,
    friction=0.8,
    speed=initial_speed,
    rotation_speed=motor_speed
)


connecting_rod = visuals.ConnectingRod(
    x=0,
    y=0,
    z=0,
    radius=cylinder_radius,
    mass=1000,
    friction=0.1
)


piston = visuals.Piston(
    x=0,
    y=0,
    z=0,
    radius=cylinder_radius,
    mass=1000,
    friction=0.1,
    height=cylinder_height,
    speed=initial_speed
)



floor.x = initial_position[0]
floor.y = initial_position[1]
floor.z = initial_position[2]
crankshaft.x = initial_position[0]
crankshaft.y = initial_position[1]
crankshaft.z = initial_position[2]
connecting_rod.x = initial_position[0]
connecting_rod.y = initial_position[1]
connecting_rod.z = initial_position[2]
piston.x = initial_position[0]
piston.y = initial_position[1]
piston.z = initial_position[2]



for i in range(simulation_duration):
    
    floor.x += initial_speed * i
    floor.y += initial_speed * i
    floor.z += initial_speed * i

    
    crankshaft.x += initial_speed * i
    crankshaft.y += initial_speed * i
    crankshaft.z += initial_speed * i

    
    connecting_rod.x += initial_speed * i
    connecting_rod.y += initial_speed * i
    connecting_rod.z += initial_speed * i

    
    piston.x += initial_speed * i
    piston.y += initial_speed * i
    piston.z += initial_speed * i

    
    floor.friction += joint_strength * (i / simulation_duration)
    crankshaft.friction += joint_strength * (i / simulation_duration)
    connecting_rod.friction += joint_strength * (i / simulation_duration)
    piston.friction += joint_strength * (i / simulation_duration)

    
    if motor_speed > 0:
        crankshaft.speed = motor_speed
        connecting_rod.speed = motor_speed
        piston.speed = motor_speed
    else:
        crankshaft.speed = 0
        connecting_rod.speed = 0
        piston.speed = 0

    
    visuals.Draw(
        floor,
        x=floor.x,
        y=floor.y,
        z=floor.z,
        color=(1, 0, 0),  
        linewidth=2
    )

    visuals.Draw(
        crankshaft,
        x=crankshaft.x,
        y=crankshaft.y,
        z=crankshaft.z,
        color=(0, 0, 1),  
        linewidth=2
    )

    visuals.Draw(
        connecting_rod,
        x=connecting_rod.x,
        y=connecting_rod.y,
        z=connecting_rod.z,
        color=(0, 0, 1),  
        linewidth=2
    )

    visuals.Draw(
        piston,
        x=piston.x,
        y=piston.y,
        z=piston.z,
        color=(1, 1, 0),  
        linewidth=2
    )

    time.sleep(0.01)  

print("Simulation complete.")