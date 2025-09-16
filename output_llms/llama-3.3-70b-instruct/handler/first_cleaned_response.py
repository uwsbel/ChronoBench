import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import rospy
from std_msgs.msg import Int32


sys = chrono.ChSystemNSC()


sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


material = chrono.ChMaterialSurfaceNSC()


floor_body = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, material)
floor_body.SetPos(chrono.ChVectorD(0, -5, 0))
floor_body.SetBodyFixed(True)
sys.Add(floor_body)


box_body = chrono.ChBodyEasyBox(sys, 1, 1, 1, 10, True, material)
box_body.SetPos(chrono.ChVectorD(0, 5, 0))
sys.Add(box_body)


class CustomROSHdlr(chrono.ChROSHandler):
    def __init__(self):
        super().__init__()
        self.pub = rospy.Publisher('custom_topic', Int32, queue_size=10)
        self.msg = Int32()

    def Update(self):
        self.msg.data = int(sys.GetChTime())
        self.pub.publish(self.msg)


rospy.init_node('pychrono_simulation')


ros_manager = chrono.ChROSManager(sys)
clock_hdlr = chrono.ChROSClockHandler(sys)
body_hdlr = chrono.ChROSBODYHandler(sys)
transform_hdlr = chrono.ChROSTransformHandler(sys)
custom_hdlr = CustomROSHdlr()
ros_manager.AddHandler(clock_hdlr)
ros_manager.AddHandler(body_hdlr)
ros_manager.AddHandler(transform_hdlr)
ros_manager.AddHandler(custom_hdlr)


time_step = 0.01
duration = 10


while sys.GetChTime() < duration:
    sys.DoStepDynamics(time_step)
    ros_manager.Update()
    rospy.sleep(time_step)


ros_manager.Shutdown()