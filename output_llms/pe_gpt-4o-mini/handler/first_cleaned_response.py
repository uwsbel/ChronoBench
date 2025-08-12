import os
import math
import numpy as np
import pychrono as chrono
import rospy
from std_msgs.msg import Int32


rospy.init_node('chrono_ros_simulation', anonymous=True)
pub = rospy.Publisher('chrono_topic', Int32, queue_size=10)


def ros_publish(value):
    msg = Int32()
    msg.data = value
    pub.publish(msg)


my_system = chrono.ChSystemNSC()
gravity = chrono.ChVector3d(0, -9.81, 0)
my_system.Set_G_acc(gravity)


floor_material = chrono.ChMaterialSurfaceNSC()
floor_material.SetFriction(0.5)
floor = chrono.ChBody()
floor.SetPos(chrono.ChVector3d(0, -0.5, 0))
floor.SetMass(0)  
floor.SetMaterialSurface(floor_material)
floor_shape = chrono.ChVisualShapeBox(10, 1, 10)
floor.AddVisualShape(floor_shape)
my_system.Add(floor)


box_material = chrono.ChMaterialSurfaceNSC()
box_material.SetFriction(0.5)
box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, box_material)
box.SetPos(chrono.ChVector3d(0, 0, 0))
my_system.Add(box)


class ROSHandler:
    def __init__(self):
        self.prev_time = rospy.get_time()

    def update(self):
        current_time = rospy.get_time()
        elapsed_time = current_time - self.prev_time
        self.prev_time = current_time
        return elapsed_time

ros_handler = ROSHandler()


rate = rospy.Rate(10)  
while not rospy.is_shutdown():
    
    my_system.DoStepDynamics(0.01)

    
    elapsed_time = ros_handler.update()
    ros_publish(int(elapsed_time * 1000))  

    
    

    rate.sleep()

print("Simulation finished.")