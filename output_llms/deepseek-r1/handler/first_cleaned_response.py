import pychrono as chrono
import pychrono.ros as chrono_ros
import time
import rospy
from std_msgs.msg import Int32

class CustomHandler(chrono_ros.ChROSHandler):
    def __init__(self):
        super().__init__()
        self.publisher = None
        self.counter = 0

    def Initialize(self):
        self.publisher = rospy.Publisher("/custom_data", Int32, queue_size=10)
        return True

    def Tick(self, sim_time):
        msg = Int32()
        msg.data = self.counter
        self.publisher.publish(msg)
        self.counter += 1
        return True


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.5)
material.SetDampingF(0.2)


floor = chrono.ChBodyEasyBox(10, 0.5, 10, 1000, True, True, material)
floor.SetPos(chrono.ChVectorD(0, -0.25, 0))
floor.SetBodyFixed(True)
system.Add(floor)


box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, material)
box.SetPos(chrono.ChVectorD(0, 1, 0))
system.Add(box)


ros_manager = chrono_ros.ChROSPythonManager(system)
ros_manager.AddHandler(chrono_ros.ChROSClockHandler())
ros_manager.AddHandler(chrono_ros.ChROSBodyHandler(box, "/box_state"))
ros_manager.AddHandler(chrono_ros.ChROSTFHandler())
ros_manager.AddHandler(CustomHandler())


time_step = 0.01
simulation_duration = 10.0
real_time_start = time.time()


while system.GetChTime() < simulation_duration:
    
    system.DoStepDynamics(time_step)
    
    
    ros_manager.Update(system.GetChTime(), time_step)
    
    
    real_time_elapsed = time.time() - real_time_start
    desired_real_time = system.GetChTime()
    sleep_time = desired_real_time - real_time_elapsed
    if sleep_time > 0:
        time.sleep(sleep_time)