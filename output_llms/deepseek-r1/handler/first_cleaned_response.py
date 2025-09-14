import pychrono as chrono
import pychrono.ros as chrono_ros
import rospy
from std_msgs.msg import Int32

class CustomHandler(chrono_ros.ChROSHandler):
    def __init__(self, topic_name):
        super().__init__()
        self.topic_name = topic_name
        self.publisher = None
        self.counter = 0

    def Initialize(self, ros_manager):
        self.publisher = ros_manager.GetNode().advertise(self.topic_name, Int32, 10)
        return True

    def Tick(self, time):
        msg = Int32()
        msg.data = self.counter
        self.publisher.publish(msg)
        self.counter += 1
        return True


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, -9.81))


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.5)
material.SetRestitution(0.1)


floor = chrono.ChBodyEasyBox(10, 10, 0.1, 1000, True, True, material)
floor.SetPos(chrono.ChVectorD(0, 0, -0.1))
floor.SetBodyFixed(True)
system.Add(floor)


box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True, material)
box.SetPos(chrono.ChVectorD(0, 0, 2))
system.Add(box)


manager = chrono_ros.ChROSManager()
manager.AttachSystem(system)


manager.AddHandler(chrono_ros.ChROSClockHandler())
manager.AddHandler(chrono_ros.ChROSBodyHandler())
manager.AddHandler(chrono_ros.ChROSTransformHandler())


custom_handler = CustomHandler("/chrono/counter")
manager.AddHandler(custom_handler)


manager.Initialize()


step_size = 0.01
realtime_step = True

while manager.IsOk():
    
    system.DoStepDynamics(step_size)
    
    
    manager.Advance(step_size, realtime_step)


manager.Finalize()