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


floor = chrono.ChBodyEasyBox(100, 1, 100, 1000, True, True, material)
my_system.Add(floor)


box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, material)
box.SetPos(chrono.ChVectorD(0, 1, 0))
my_system.Add(box)


force = chrono.ChForce()
force.SetMode(chrono.ChForce.FORCE)
force.SetF_x(chrono.ChVectorD(100, 0, 0))
box.AddForce(force)


rospy.init_node('pychrono_ros')


class CustomHandler(chrono.ChRosBase):
    def __init__(self):
        super().__init__()
        self.pub = rospy.Publisher('custom_topic', Int32, queue_size=10)

    def Update(self, time, dt):
        msg = Int32()
        msg.data = int(time * 1000)  
        self.pub.publish(msg)


ros_manager = chrono.ChRosManager()


ros_manager.AddHandler(chrono.ChRosClockHandler())


ros_manager.AddHandler(chrono.ChRosBodyHandler())


ros_manager.AddHandler(chrono.ChRosTransformHandler())


ros_manager.AddHandler(CustomHandler())


while rospy.is_shutdown() is False:
    
    my_system.DoStepDynamics(0.01)

    
    ros_manager.Update(my_system.GetChTime(), 0.01)

    
    rospy.sleep(0.01)