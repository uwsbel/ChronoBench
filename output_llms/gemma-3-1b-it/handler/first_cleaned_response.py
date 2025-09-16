import time
import numpy as np
from pychrono import Chrono, System
from pychrono.utils import get_time_unit
import rospy
from std_msgs.msg import Float64


gravity = 9.81  
floor_mass = 1000  
box_mass = 200  
box_size = 1.0  
box_position = [0, 0]  
box_velocity = [0, 0]  
simulation_duration = 10  
timestep = 0.01  


system = Chrono(
    system_name="MySimulation",
    gravity=gravity,
    material_properties={"density": 2700, "elastic_modulus": 2.1e9},  
    body_type="box",
    body_position=[box_position],
    body_velocity=[box_velocity],
    simulation_duration=simulation_duration,
    
)








floor = System()
floor.add_object(system)
floor.set_position([0, 0, 0])


box = System()
box.add_object(system)
box.set_position([box_position[0], box_position[1], box_position[2]])
box.set_velocity([0, 0])



def publish_message(message):
    rospy.loginfo("Publishing message: %s", message)
    rospy.message_path("MySimulation_Update") 


rospy.init_node('MySimulation')
rospy.loginfo("ROS Manager initialized.")


while True:
    
    system.update()

    
    publish_message(system.update())

    
    time.sleep(timestep)