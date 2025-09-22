import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.robot as robosimian
import rospy
from std_msgs.msg import Int32


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))


mysystem = chrono.ChSystemSMC()


mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.4)
material.SetDampingF(0.000001)
material.SetCompliance(0.000001)
material.SetComplianceT(0.000001)


floor = chrono.ChBodyEasyBox(mysystem, 4, 1, 4, 1000, True, True, material)
floor.SetPos(chrono.ChVectorD(0, -1, 0))
floor.SetBodyFixed(True)


box = chrono.ChBodyEasyBox(mysystem, 1, 1, 1, 1000, True, True, material)
box.SetPos(chrono.ChVectorD(0, 2, 0))
box.SetBodyFixed(False)


rospy.init_node('pychrono_ros')


class MyROSHdlr:
    def __init__(self):
        self.pub = rospy.Publisher('my_topic', Int32, queue_size=10)
        self.count = 0

    def Advance(self, mstep):
        self.count += 1
        self.pub.publish(self.count)


ros_manager = veh.ChROSManager()
ros_manager.SetRate(100)  


clock_hdlr = veh.ChROSClockHandler(mysystem, ros_manager)
ros_manager.RegisterHandler(clock_hdlr)


body_hdlr = veh.ChROSBODYHandler(mysystem, ros_manager)
ros_manager.RegisterHandler(body_hdlr)


transform_hdlr = veh.ChROSTRANSFORMHandler(mysystem, ros_manager)
ros_manager.RegisterHandler(transform_hdlr)


my_hdlr = MyROSHdlr()
ros_manager.RegisterHandler(my_hdlr)


mstep = chrono.ChTimeStepperType.Default
mysystem.SetChTimeStepperType(mstep)
m_tstep = 0.001
m_tend = 10

while mysystem.GetChTime() < m_tend:
    mysystem.DoStepDynamics(m_tstep)
    ros_manager.Advance(m_tstep)

    
    if ros_manager.IsRealtime():
        chrono.ChRealtimeStep(m_tstep)


rospy.signal_shutdown('simulation finished')