import pychrono.core as chrono
import pychrono_ros as ros
import rospy
from std_msgs.msg import Int32


chrono.SetChronoDataPath('/usr/local/share/chrono/data/')


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.5)
material.SetRestitution(0.2)
system.SetMaterialSurface(material)


floor = chrono.ChBodyEasyBox(10, 1, 10, 1000, True)
floor.SetPos(chrono.ChVectorD(0, -1, 0))
system.Add(floor)


box = chrono.ChBodyEasyBox(1, 1, 1, 1, False)
box.SetPos(chrono.ChVectorD(0, 1, 0))
system.Add(box)


class CustomHandler(ros.ChRosBase):
    def __init__(self):
        super().__init__()
        self.publisher = rospy.Publisher('custom_topic', Int32, queue_size=10)

    def advance(self, time):
        msg = Int32()
        msg.data = int(time)
        self.publisher.publish(msg)


ros_manager = ros.ChRosSystem(system)


ros_manager.AddHandler(ros.ChRosClockHandler())
ros_manager.AddHandler(ros.ChRosBodyHandler())
ros_manager.AddHandler(ros.ChRosTransformHandler())
ros_manager.AddHandler(CustomHandler())


rospy.init_node('pychrono_ros_node')


while True:
    
    system.DoStepDynamics(0.01)

    
    ros_manager.Advance(0.01)

    
    chrono.ChUtils.Sleep(0.01)