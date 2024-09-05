import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import rospy
from std_msgs.msg import Int32
import numpy as np


chrono.SetChronoDataPath('/usr/local/share/chrono/data/')


rospy.init_node('pychrono_simulation')


sys = chrono.ChSystemNSC()


sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


floor_mat = chrono.ChMaterialSurfaceNSC()
floor_mat.SetFriction(0.5)
floor_mat.SetRestitution(0.5)
floor_body = chrono.ChBodyEasyBox(sys,  
                                  10, 10, 1,  
                                  1000,  
                                  True,  
                                  floor_mat)  
sys.Add(floor_body)


box_mat = chrono.ChMaterialSurfaceNSC()
box_mat.SetFriction(0.5)
box_mat.SetRestitution(0.5)
box_body = chrono.ChBodyEasyBox(sys,  
                                1, 1, 1,  
                                1000,  
                                False,  
                                box_mat)  
sys.Add(box_body)


class CustomROSHAndler(chrono.ChROSHAndler):
    def __init__(self):
        super().__init__()
        self.pub = rospy.Publisher('custom_topic', Int32, queue_size=10)

    def Advance(self, time):
        msg = Int32()
        msg.data = int(time * 1000)  
        self.pub.publish(msg)


ros_manager = chrono.ChROSMAndler(sys)


ros_manager.AddHandler(chrono.ChROSClockHandler())


ros_manager.AddHandler(chrono.ChROSBodyHandler())


ros_manager.AddHandler(chrono.ChROSTransformHandler())


ros_manager.AddHandler(CustomROSHAndler())


time_step = 0.01
while not rospy.is_shutdown():
    
    sys.DoStepDynamics(time_step)

    
    ros_manager.Advance(sys.GetChTime())

    
    rospy.sleep(time_step)