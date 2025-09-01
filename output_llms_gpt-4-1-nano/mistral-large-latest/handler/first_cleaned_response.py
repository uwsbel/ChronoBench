import pychrono as chrono
import pychrono.robot as chrono_robot
import rospy
from std_msgs.msg import Int32
import time


rospy.init_node('chrono_ros_simulation', anonymous=True)


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.3)


floor = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True, material)
floor.SetPos(chrono.ChVectorD(0, -1, 0))
floor.SetBodyFixed(True)
system.Add(floor)


box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, material)
box.SetPos(chrono.ChVectorD(0, 1, 0))
system.Add(box)


class CustomROSHandler(chrono_robot.ChROSHandler):
    def __init__(self, topic_name):
        super().__init__()
        self.publisher = rospy.Publisher(topic_name, Int32, queue_size=10)

    def step(self, time):
        
        msg = Int32()
        msg.data = int(time)
        self.publisher.publish(msg)


ros_manager = chrono_robot.ChROSManager(system)


ros_manager.add_handler(chrono_robot.ChROSClockHandler())
ros_manager.add_handler(chrono_robot.ChROSBodyHandler())
ros_manager.add_handler(chrono_robot.ChROSTransformHandler())


custom_handler = CustomROSHandler('/chrono_ros_topic')
ros_manager.add_handler(custom_handler)


ros_manager.initialize()


simulation_time = 10  
step_size = 0.01  

start_time = time.time()

while rospy.is_shutdown() is False and system.GetChTime() < simulation_time:
    
    system.DoStepDynamics(step_size)

    
    ros_manager.update()

    
    real_time = time.time() - start_time
    if real_time < system.GetChTime():
        time.sleep(system.GetChTime() - real_time)


rospy.signal_shutdown('Simulation completed')