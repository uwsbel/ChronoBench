import pychrono as chrono
import rospy
from std_msgs.msg import Int32


chrono.SetChronoThread(chrono.CHRONO_THREAD_ROS)


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.5)
material.SetRestitution(0.2)


floor = chrono.ChBodyEasyBox(10, 10, 0.1, material)
floor.SetPos(chrono.ChVectorD(0, 0, -0.5))
floor.SetBodyFixed(True)
system.AddBody(floor)


box = chrono.ChBodyEasyBox(1, 1, 1, material)
box.SetPos(chrono.ChVectorD(0, 0, 0.5))
system.AddBody(box)


class MyRosHandler(chrono.ChRosHandler):
    def __init__(self, topic_name):
        super().__init__()
        self.topic_name = topic_name
        self.pub = rospy.Publisher(self.topic_name, Int32, queue_size=10)

    def OnUpdate(self, time):
        
        msg = Int32()
        msg.data = int(time)
        self.pub.publish(msg)


ros_manager = chrono.ChRosManager()
ros_manager.SetClockHandler(chrono.ChRosClockHandler())
ros_manager.SetBodyHandler(chrono.ChRosBodyHandler())
ros_manager.SetTransformHandler(chrono.ChRosTransformHandler())
ros_manager.AddHandler(MyRosHandler("my_topic"))


rospy.init_node("chrono_ros_node")
ros_manager.Start()

while not rospy.is_shutdown():
    system.DoStepDynamics(0.01)
    ros_manager.DoStep()

ros_manager.Stop()