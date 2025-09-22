import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import numpy as np
import rospy
from std_msgs.msg import Int32


sys = chrono.ChSystemNSC()


sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


floor = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, True)
floor.SetPos(chrono.ChVectorD(0, -5, 0))
floor.SetBodyFixed(True)
sys.Add(floor)


box = chrono.ChBodyEasyBox(sys, 1, 1, 1, 10, True, True)
box.SetPos(chrono.ChVectorD(0, 5, 0))
sys.Add(box)


class CustomROSHandler:
    def __init__(self):
        self.pub = rospy.Publisher('custom_topic', Int32, queue_size=10)
        self.count = 0

    def Update(self):
        self.count += 1
        self.pub.publish(self.count)


rospy.init_node('pychrono_simulation')


ros_manager = chrono.ChROSManager(sys)


clock_handler = chrono.ChROSClockHandler(ros_manager)


body_handler = chrono.ChROSBODYHandler(ros_manager)


transform_handler = chrono.ChROSTransformHandler(ros_manager)


custom_handler = CustomROSHandler()
ros_manager.RegisterHandler(custom_handler)


while not rospy.is_shutdown():
    sys.DoStepDynamics(0.01)
    ros_manager.Update()
    rospy.sleep(0.01)