import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as irrlicht
import pychrono.vehicle as vehicle
import rospy
from std_msgs.msg import Int32

# Set Chrono data path
chrono.SetChronoDataPath(os.path.join(os.getcwd(), 'chrono_data'))

# Initialize ROS node
rospy.init_node('pychrono_ros_simulation')

# Initialize Chrono system
my_system = chrono.ChSystemNSC()
my_system.Set_Gravity(chrono.ChVector3d(0, -9.81, 0))

# Create and add floor to the system
floor_mat = chrono.ChMaterialSurfaceNSC()
floor_mat.SetFriction(0.6)
floor = chrono.ChBody()
floor.SetBodyFixed(True)
floor.SetPos(chrono.ChVector3d(0, -1, 0))
floor_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(10, 0.1, 10))
floor.AddVisualShape(floor_shape)
floor.SetMaterialSurface(floor_mat)
my_system.Add(floor)

# Create and add box to the system
box_mat = chrono.ChMaterialSurfaceNSC()
box_mat.SetFriction(0.6)
box = chrono.ChBody()
box.SetPos(chrono.ChVector3d(0, 1, 0))
box_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(0.5, 0.5, 0.5))
box.AddVisualShape(box_shape)
box.SetMaterialSurface(box_mat)
my_system.Add(box)

# Set up ROS manager
ros_manager = robot.RosManager()
ros_manager.SetClockHandler(robot.RosClockHandler())
ros_manager.SetBodyHandler(robot.RosBodyHandler())
ros_manager.SetTransformHandler(robot.RosTransformHandler())

# Create custom ROS handler
class CustomRosHandler(robot.RosHandler):
    def __init__(self):
        super(CustomRosHandler, self).__init__()
        self.pub = rospy.Publisher('/pychrono/custom_topic', Int32, queue_size=10)

    def Publish(self, msg):
        self.pub.publish(msg)

# Register custom ROS handler
ros_manager.RegisterHandler(CustomRosHandler())

# Set up visualization
vis = irrlicht.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('PyChrono ROS Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()

# Simulation loop
while vis.Run():
    # Advance the system
    my_system.DoStepDynamics(0.01)

    # Update ROS communication
    ros_manager.Update()

    # Publish custom ROS message
    custom_msg = Int32()
    custom_msg.data = int(box.GetPos().x)
    ros_manager.GetHandler(CustomRosHandler()).Publish(custom_msg)

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()