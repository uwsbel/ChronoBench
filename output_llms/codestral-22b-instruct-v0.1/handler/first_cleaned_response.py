import pychrono as chrono
import pychrono.ros as chrono_ros
import rospy
from std_msgs.msg import Int32


chrono.SetChronoDataPath("/usr/local/share/chrono/data/")
my_system = chrono.ChSystemNSC()


my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


material = chrono.ChMaterialSurfaceNSC()


floor_body = chrono.ChBodyEasyBox(10, 1, 10, 1000, material)
floor_body.SetBodyFixed(True)
my_system.Add(floor_body)


box_body = chrono.ChBodyEasyBox(1, 1, 1, 10, material)
box_body.SetPos(chrono.ChVectorD(0, 2, 0))
my_system.Add(box_body)


class CustomROSHandler(chrono_ros.ChROSHandler):
    def __init__(self, topic_name):
        self.publisher = rospy.Publisher(topic_name, Int32, queue_size=10)

    def Update(self, system):
        
        msg = Int32()
        msg.data = 123  
        self.publisher.publish(msg)


ros_manager = chrono_ros.ChROSManager(my_system)
ros_manager.AddClockHandler()
ros_manager.AddBodyHandler(floor_body)
ros_manager.AddBodyHandler(box_body)


custom_handler = CustomROSHandler("custom_topic")
ros_manager.AddHandler(custom_handler)


time_step = 0.01
while rospy.is_shutdown() is False:
    my_system.DoStepDynamics(time_step)
    ros_manager.Update()
    chrono.ChTime.Sleep(time_step)