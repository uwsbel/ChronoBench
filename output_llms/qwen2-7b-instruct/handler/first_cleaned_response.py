import pychrono as chrono
from pychrono importchronoengine
import rospy
from std_msgs.msg import Int32


chronoengine.Initialize()


gravity = chrono.ChVectorD(0, -9.81, 0)  
material = chrono.ChMaterialSurfaceNSC()  


chrono_system = chrono.ChSystemNSC()
chrono_system.Set_G_acc(gravity)


floor = chrono.ChBodyEasyBox(10, 10, 0.1, 1000, True, True, material)
floor.SetPos(chrono.ChVectorD(0, -0.5, 0))
chrono_system.Add(floor)


box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True, material)
box.SetPos(chrono.ChVectorD(0, 0, 0.1))
chrono_system.Add(box)


rospy.init_node('pychrono_ros_node', anonymous=True)
rate = rospy.Rate(10)  


class CustomROSHandler:
    def __init__(self):
        self.pub = rospy.Publisher('my_topic', Int32, queue_size=10)
        self.counter = 0

    def publish_message(self):
        self.counter += 1
        self.pub.publish(self.counter)


clock_handler = chrono.ChSystemClock()
body_handler = chrono.ChBody()
transform_handler = chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT)
ros_manager = chrono.ChRONOSManager()
ros_manager.AddHandler(clock_handler)
ros_manager.AddHandler(body_handler)
ros_manager.AddHandler(transform_handler)
ros_manager.AddHandler(CustomROSHandler().publish_message)


while not rospy.is_shutdown():
    chrono_system.DoStepDynamics(1.0/100.0)
    ros_manager.Advance(1.0/100.0)
    rate.sleep()


chronoengine.Finalize()