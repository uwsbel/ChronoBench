import pychrono as chrono
import pychrono.ros as chrono_ros
import rospy
from std_msgs.msg import Int32


rospy.init_node('chrono_ros_simulation')


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


collision_system = chrono.ChCollisionSystemBullet()
system.SetCollisionSystem(collision_system)


floor_body = chrono.ChBodyEasyBox(10, 0.5, 10, 1000, True, True)
floor_body.SetPos(chrono.ChVectorD(0, -1, 0))
floor_body.SetBodyFixed(True)
floor_body.GetVisualModel().SetColor(chrono.ChColor(0.5, 0.5, 0.5))
system.Add(floor_body)


box_body = chrono.ChBodyEasyBox(1, 1, 1, 10, True, True)
box_body.SetPos(chrono.ChVectorD(0, 2, 0))
box_body.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
box_body.GetVisualModel().SetColor(chrono.ChColor(0.8, 0.2, 0.2))
system.Add(box_body)


class CustomIntPublisher(chrono_ros.ChRosGenericHandler):
    def __init__(self, topic_name, system):
        super().__init__()
        self.pub = rospy.Publisher(topic_name, Int32, queue_size=10)
        self.counter = 0
        self.system = system

    def update(self, time):
        
        msg = Int32()
        msg.data = self.counter
        self.pub.publish(msg)
        self.counter += 1


ros_manager = chrono_ros.ChRosManager()
ros_manager.set_system(system)


ros_manager.add_handler(chrono_ros.ChRosClockHandler())
ros_manager.add_handler(chrono_ros.ChRosBodyHandler())
ros_manager.add_handler(chrono_ros.ChRosTransformHandler())


custom_handler = CustomIntPublisher('/chrono_counter', system)
ros_manager.add_handler(custom_handler)


time_step = 0.01
sim_time = 10.0


while rospy.is_shutdown() is False:
    
    ros_manager.update()

    
    system.DoStepDynamics(time_step)

    
    try:
        rospy.sleep(time_step)
    except rospy.ROSInterruptException:
        break

    
    if system.GetChTime() >= sim_time:
        break

print("Simulation completed")