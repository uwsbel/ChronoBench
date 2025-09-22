import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.robot as robosimian
import rospy
from std_msgs.msg import Int32

# Initialize the PyChrono environment
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))

# Create the Chrono system
mysystem = chrono.ChSystemSMC()

# Set the gravitational acceleration
mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the physical material properties
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.4)
material.SetDampingF(0.000001)
material.SetCompliance(0.000001)
material.SetComplianceT(0.000001)

# Create the floor
floor = chrono.ChBodyEasyBox(mysystem, 4, 1, 4, 1000, True, True, material)
floor.SetPos(chrono.ChVectorD(0, -1, 0))
floor.SetBodyFixed(True)

# Create the box
box = chrono.ChBodyEasyBox(mysystem, 1, 1, 1, 1000, True, True, material)
box.SetPos(chrono.ChVectorD(0, 2, 0))
box.SetBodyFixed(False)

# Create the ROS node
rospy.init_node('pychrono_ros')

# Create a custom ROS handler
class MyROSHdlr:
    def __init__(self):
        self.pub = rospy.Publisher('my_topic', Int32, queue_size=10)
        self.count = 0

    def Advance(self, mstep):
        self.count += 1
        self.pub.publish(self.count)

# Create the ROS manager
ros_manager = veh.ChROSManager()
ros_manager.SetRate(100)  # Hz

# Create the clock handler
clock_hdlr = veh.ChROSClockHandler(mysystem, ros_manager)
ros_manager.RegisterHandler(clock_hdlr)

# Create the body handler
body_hdlr = veh.ChROSBODYHandler(mysystem, ros_manager)
ros_manager.RegisterHandler(body_hdlr)

# Create the transform handler
transform_hdlr = veh.ChROSTRANSFORMHandler(mysystem, ros_manager)
ros_manager.RegisterHandler(transform_hdlr)

# Register the custom handler
my_hdlr = MyROSHdlr()
ros_manager.RegisterHandler(my_hdlr)

# Set up the simulation loop
mstep = chrono.ChTimeStepperType.Default
mysystem.SetChTimeStepperType(mstep)
m_tstep = 0.001
m_tend = 10

while mysystem.GetChTime() < m_tend:
    mysystem.DoStepDynamics(m_tstep)
    ros_manager.Advance(m_tstep)

    # Keep the real-time synchronization
    if ros_manager.IsRealtime():
        chrono.ChRealtimeStep(m_tstep)

# Shut down the ROS node
rospy.signal_shutdown('simulation finished')