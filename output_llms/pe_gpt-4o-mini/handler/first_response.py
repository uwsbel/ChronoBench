import os
import math
import numpy as np
import pychrono as chrono
import rospy
from std_msgs.msg import Int32

# Initialize ROS
rospy.init_node('chrono_ros_simulation', anonymous=True)
pub = rospy.Publisher('chrono_topic', Int32, queue_size=10)

# Function to publish integer messages
def ros_publish(value):
    msg = Int32()
    msg.data = value
    pub.publish(msg)

# Initialize Chrono system with gravitational acceleration
my_system = chrono.ChSystemNSC()
gravity = chrono.ChVector3d(0, -9.81, 0)
my_system.Set_G_acc(gravity)

# Create fixed floor
floor_material = chrono.ChMaterialSurfaceNSC()
floor_material.SetFriction(0.5)
floor = chrono.ChBody()
floor.SetPos(chrono.ChVector3d(0, -0.5, 0))
floor.SetMass(0)  # Fixed body
floor.SetMaterialSurface(floor_material)
floor_shape = chrono.ChVisualShapeBox(10, 1, 10)
floor.AddVisualShape(floor_shape)
my_system.Add(floor)

# Create movable box
box_material = chrono.ChMaterialSurfaceNSC()
box_material.SetFriction(0.5)
box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, box_material)
box.SetPos(chrono.ChVector3d(0, 0, 0))
my_system.Add(box)

# ROS handler for simulation
class ROSHandler:
    def __init__(self):
        self.prev_time = rospy.get_time()

    def update(self):
        current_time = rospy.get_time()
        elapsed_time = current_time - self.prev_time
        self.prev_time = current_time
        return elapsed_time

ros_handler = ROSHandler()

# Simulation loop
rate = rospy.Rate(10)  # 10 Hz
while not rospy.is_shutdown():
    # Advance the Chrono simulation
    my_system.DoStepDynamics(0.01)

    # Update ROS communication
    elapsed_time = ros_handler.update()
    ros_publish(int(elapsed_time * 1000))  # Publish elapsed time in milliseconds

    # Render scene (if visualization is set up)
    # Note: Visualization setup is omitted in this basic example.

    rate.sleep()

print("Simulation finished.")