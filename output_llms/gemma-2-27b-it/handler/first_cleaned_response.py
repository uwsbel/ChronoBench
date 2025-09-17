import pychrono as chrono
import pychrono.ros as chronoros
import rospy
from std_msgs.msg import Int32


chrono.SetChronoDataPath("data/")


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.3)
material.SetRestitution(0.2)


floor = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True, material)
floor.SetPos(chrono.ChVectorD(0, -0.1, 0))
floor.SetBodyFixed(True)
system.Add(floor)


box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 10, True, True, material)
box.SetPos(chrono.ChVectorD(0, 1, 0))
system.Add(box)


rospy.init_node('chrono_ros_node')


class CustomRosHandler(chronoros.ChROSHandler):
    def __init__(self, system, topic_name):
        super().__init__(system)
        self.pub = rospy.Publisher(topic_name, Int32, queue_size=10)

    def OnUpdate(self):
        msg = Int32()
        msg.data = int(system.GetChTime())
        self.pub.publish(msg)


ros_manager = chronoros.ChROSManager(system)


ros_manager.SetClockHandler(chronoros.ChROSClockHandler())
ros_manager.SetBodyHandler(chronoros.ChROSBodyHandler())
ros_manager.SetTransformHandler(chronoros.ChROSTransformHandler())


custom_handler = CustomRosHandler(system, "/chrono_time")
ros_manager.AddHandler(custom_handler)


while not rospy.is_shutdown():
    system.DoStepDynamics(0.01)
    ros_manager.Update()
    rospy.Rate(100).sleep()