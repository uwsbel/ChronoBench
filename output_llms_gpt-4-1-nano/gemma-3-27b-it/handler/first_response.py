import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import rospy
from std_msgs.msg import Int32

class CustomROSHelper(object):
    """
    A custom ROS helper class to publish integer messages.
    """
    def __init__(self, topic_name):
        self.topic_name = topic_name
        self.publisher = rospy.Publisher(self.topic_name, Int32, queue_size=10)

    def publish_message(self, value):
        message = Int32()
        message.data = value
        self.publisher.publish(message)

def main():
    """
    Main function to create and run the PyChrono simulation with ROS integration.
    """
    # Initialize ROS node
    rospy.init_node('pychrono_ros_sim', anonymous=True)

    # Set Chrono data path
    chrono.SetChronoDataPath("../chrono/data/")

    # Create a Chrono system
    sys = chrono.ChSystemNSC()

    # Set gravity
    sys.Set_G_acc(chrono.ChVector3d(0, -9.81, 0))

    # Create a physical material
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(0.4)
    material.SetRestitution(0.2)

    # Create a fixed floor
    body_floor = chrono.ChBody()
    body_floor.SetBodyFixed(True)
    body_floor.SetPos(chrono.ChVector3d(0, -1, 0))
    body_floor.SetShapeFromBox(10, 1, 10)
    body_floor.SetMaterial(material)
    sys.Add(body_floor)

    # Create a movable box
    body_box = chrono.ChBody()
    body_box.SetPos(chrono.ChVector3d(0, 1, 0))
    body_box.SetShapeFromBox(1, 1, 1)
    body_box.SetMaterial(material)
    body_box.SetMass(1)
    sys.Add(body_box)

    # Create a custom ROS handler
    ros_helper = CustomROSHelper("chrono_data")

    # Create a ROS manager
    ros_manager = chrono.ChRosManager()
    ros_manager.SetClockHandler(True)
    ros_manager.SetBodyHandler(True)
    ros_manager.SetTransformHandler(True)
    ros_manager.RegisterHelper(ros_helper)
    ros_manager.Initialize(sys)

    # Visualization setup (optional)
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('PyChrono with ROS')
    vis.Initialize()
    vis.AddCamera(chrono.ChVector3d(0, 5, -10))
    vis.AddTypicalLights()

    # Simulation loop
    time_step = 0.01
    while vis.Run():
        # Advance the simulation
        sys.DoStepDynamics(time_step)

        # Publish data to ROS
        ros_manager.Update()
        ros_helper.publish_message(int(body_box.GetPos().y * 100)) # Publish box y position

        # Visualization update
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

if __name__ == "__main__":
    main()