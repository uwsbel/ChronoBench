import pychrono as chrono
import pychrono.ros as chr_ros
import rospy
from std_msgs.msg import Int32


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.5)
material.SetRestitution(0.1)


floor = chrono.ChBodyEasyBox(20, 1, 20, 1000, True, True, material)
floor.SetBodyFixed(True)
floor.SetPos(chrono.ChVectorD(0, -0.5, 0))
system.Add(floor)


box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, material)
box.SetPos(chrono.ChVectorD(0, 5, 0))
system.Add(box)


rospy.init_node('chrono_simulation', anonymous=True)


class CustomROSHandler(chr_ros.ChROSHandler):
    def __init__(self):
        super().__init__()
        self.pub = rospy.Publisher('custom_topic', Int32, queue_size=10)
        self.counter = 0

    def PublishCallback(self):
        msg = Int32()
        msg.data = self.counter
        self.pub.publish(msg)
        self.counter += 1


ros_manager = chr_ros.ChROSManager()


clock_handler = chr_ros.ChROSClockHandler()
ros_manager.AddHandler(clock_handler)

body_handler = chr_ros.ChROSBodyHandler(system)
ros_manager.AddHandler(body_handler)

transform_handler = chr_ros.ChROSTransformHandler()
ros_manager.AddHandler(transform_handler)

custom_handler = CustomROSHandler()
ros_manager.AddHandler(custom_handler)


ros_manager.Initialize()


step_size = 0.01
simulation_time = 0.0
real_time_factor = 1.0

while not rospy.is_shutdown():
    
    system.DoStepDynamics(step_size)
    
    
    ros_manager.Synchronize(simulation_time)
    
    
    simulation_time += step_size
    
    
    rospy.sleep(step_size / real_time_factor)


ros_manager.Finalize()