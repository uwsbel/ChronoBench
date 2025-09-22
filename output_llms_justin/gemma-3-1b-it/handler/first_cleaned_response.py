import pychrono
import numpy as np
import rospy
from pychrono.core import Chrono, System, PhysicalSystem, Force, Body, Transform


gravity = 9.81  
time_step = 0.01  
simulation_duration = 10  
simulation_rate = 10  
num_timesteps = int(simulation_duration * simulation_rate)


system = Chrono.System()
system.name = "MySimulation"
system.description = "A simple simulation with a floor and a movable box."


floor = PhysicalSystem()
floor.name = "Floor"
floor.material = "Wood"  
floor.mass = 10.0  
floor.friction = 0.1  


box = PhysicalSystem()
box.name = "Box"
box.material = "Steel"
box.mass = 5.0  
box.friction = 0.2  
box.position = [0, 0, 0]  



box.position[0] = 0.0
box.position[1] = 0.0
box.position[2] = 0.0
box.velocity[0] = 0.0
box.velocity[1] = 0.0
box.velocity[2] = 0.0


rospy.init_node('my_simulation')
rospy.loginfo("ROS initialized.")


try:
    chrono = Chrono(system, floor, box)
    chrono.set_time_step(time_step)
    chrono.set_simulation_duration(simulation_duration)
    chrono.set_rate(simulation_rate)
    chrono.set_name(simulation_duration)
    chrono.set_description(simulation_duration)
    chrono.set_body(body=body=box) 
    chrono.set_transform(transform=Transform(
        position=box.position,
        velocity=box.velocity
    ))
    chrono.set_gravity(gravity)

    
    rospy.loginfo("ROS handler registered.")

    
    rospy.sleep(simulation_rate)

    
    rospy.loginfo("Simulating...")
    rospy.loginfo("Simulation loop initiated.")

    
    box.velocity[0] = 0.1
    box.velocity[1] = 0.05
    box.velocity[2] = 0.0
    rospy.loginfo("Box velocity updated.")

    
    for _ in range(num_timesteps):
        
        box.position[0] += box.velocity[0]
        box.position[1] += box.velocity[1]
        box.position[2] += box.velocity[2]

        
        print("Box position: ", box.position)

except rospy.ROSInterruptException:
    print("Simulation interrupted.")