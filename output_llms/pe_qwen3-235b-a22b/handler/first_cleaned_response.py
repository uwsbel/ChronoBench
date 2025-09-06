import pychrono as chrono
import pychrono_ros as chrono_ros
import rospy
from std_msgs.msg import Int32


my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


floor = chrono.ChBody()
floor.SetBodyFixed(True)
floor.SetPos(chrono.ChVectorD(0, -5, 0))
floor_shape = chrono.ChVisualShapeBox(chrono.ChVectorD(10, 1, 10))
floor.AddVisualShape(floor_shape)
my_system.Add(floor)


box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True)
box.SetPos(chrono.ChVectorD(0, 0, 0))
my_system.Add(box)


ros_manager = chrono_ros.ChROSManager()
ros_manager.Initialize()


ros_manager.AddHandler(chrono_ros.ChROSClockHandler())  
ros_manager.AddHandler(chrono_ros.ChROSBodyHandler(my_system, box, "box_state"))  
ros_manager.AddHandler(chrono_ros.ChROSTransformHandler())  


class IntPublisherHandler(chrono_ros.ChROSHandler):
    def __init__(self):
        super().__init__("int_publisher")
        self.publisher = None
        self.count = 0

    def OnInitialize(self):
        self.publisher = self.node_handle.advertise("chrono_int", Int32, queue_size=10)

    def OnUpdate(self, time):
        if self.publisher:
            msg = Int32()
            msg.data = self.count
            self.publisher.publish(msg)
            self.count += 1


ros_manager.AddHandler(IntPublisherHandler())


time_step = 0.01
while ros_manager.Ok():
    my_system.DoStepDynamics(time_step)
    ros_manager.Update()