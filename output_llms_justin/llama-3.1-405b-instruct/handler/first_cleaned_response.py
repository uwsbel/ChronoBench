import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import rospy
from std_msgs.msg import Int32


chrono.SetChronoDataPath('/usr/local/share/chrono/data/')
my_system = chrono.ChSystemNSC()


my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.5)
material.SetRestitution(0.5)


floor_body = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True, material)
floor_body.SetPos(chrono.ChVectorD(0, -1, 0))
floor_body.SetBodyFixed(True)
my_system.Add(floor_body)


box_body = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, material)
box_body.SetPos(chrono.ChVectorD(0, 1, 0))
my_system.Add(box_body)


class CustomHandler(chrono.ChRosHandler):
    def __init__(self, topic_name):
        self.topic_name = topic_name
        self.publisher = rospy.Publisher(topic_name, Int32, queue_size=10)

    def Advance(self, time):
        self.publisher.publish(Int32(int(time)))


rospy.init_node('chrono_ros_node')


ros_manager = chrono.ChRosManager()
ros_manager.AddHandler(CustomHandler('/chrono/time'))
ros_manager.AddClockHandler()
ros_manager.AddBodyHandler()
ros_manager.AddTransformHandler()


time_step = 0.01
while rospy.is_shutdown() is not True:
    my_system.DoStepDynamics(time_step)
    ros_manager.Advance(my_system.GetChTime())
    rospy.sleep(time_step)