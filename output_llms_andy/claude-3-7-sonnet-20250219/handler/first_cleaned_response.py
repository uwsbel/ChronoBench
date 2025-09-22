import pychrono as chrono
import pychrono.ros as chronoros
import math
import time
import rospy
from std_msgs.msg import Int32


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.5)
material.SetRestitution(0.2)


floor = chrono.ChBodyEasyBox(10, 0.2, 10, 1000, True, True, material)
floor.SetPos(chrono.ChVectorD(0, -0.1, 0))  
floor.SetBodyFixed(True)  
system.Add(floor)


box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True, material)
box.SetPos(chrono.ChVectorD(0, 1, 0))  
box.SetName("movable_box")  
system.Add(box)


if not rospy.is_initialized():
    rospy.init_node("pychrono_simulation")


class ChROSCustomIntHandler(chronoros.ChROSHandler):
    def __init__(self, publisher):
        chronoros.ChROSHandler.__init__(self)
        self.counter = 0
        self.publisher = publisher
        
    def UpdateHandler(self):
        msg = Int32()
        msg.data = self.counter
        self.publisher.publish(msg)
        self.counter += 1
        return True


int_publisher = rospy.Publisher("chrono_counter", Int32, queue_size=10)


clock_handler = chronoros.ChROSClockHandler()
body_handler = chronoros.ChROSBodyHandler(box)
tf_handler = chronoros.ChROSTransformHandler()


custom_handler = ChROSCustomIntHandler(int_publisher)


ros_manager = chronoros.ChROSManager(system)


ros_manager.AddHandler(clock_handler)
ros_manager.AddHandler(body_handler)
ros_manager.AddHandler(tf_handler)
ros_manager.AddHandler(custom_handler)


step_size = 0.005  
realtime_factor = 1.0  


ros_manager.Initialize()


try:
    next_time = time.time()
    while not rospy.is_shutdown():
        
        current_time = time.time()
        if current_time < next_time:
            time.sleep(next_time - current_time)
        next_time = current_time + step_size * realtime_factor
        
        
        system.DoStepDynamics(step_size)
        
        
        ros_manager.UpdateHandlers()
        
except KeyboardInterrupt:
    pass


ros_manager.Shutdown()
print("Simulation completed")