import pychrono as chrono
import pychrono.ros as chrono_ros
import rospy
from std_msgs.msg import Int32


class CustomIntPublisher(chrono_ros.ChRosGenericHandler):
    def __init__(self, topic_name, publish_rate):
        super().__init__(publish_rate)
        self.pub = rospy.Publisher(topic_name, Int32, queue_size=10)
        self.counter = 0

    def update(self, time):
        msg = Int32()
        msg.data = self.counter
        self.pub.publish(msg)
        self.counter += 1
        return True

def main():
    
    rospy.init_node('chrono_ros_simulation')

    
    my_system = chrono.ChSystemNSC()
    my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  

    
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(0.4)
    material.SetRestitution(0.2)

    
    floor = chrono.ChBodyEasyBox(10, 0.5, 10, 1000, True, True, material)
    floor.SetPos(chrono.ChVectorD(0, -1, 0))
    floor.SetBodyFixed(True)
    my_system.Add(floor)

    
    box = chrono.ChBodyEasyBox(1, 1, 1, 10, True, True, material)
    box.SetPos(chrono.ChVectorD(0, 2, 0))
    my_system.Add(box)

    
    ros_manager = chrono_ros.ChRosManager()

    
    clock_handler = chrono_ros.ChRosClockHandler(10)  
    ros_manager.AddHandler(clock_handler)

    body_handler = chrono_ros.ChRosBodyHandler(box, "box", 10)  
    ros_manager.AddHandler(body_handler)

    transform_handler = chrono_ros.ChRosTransformHandler(box, "box_transform", 10)  
    ros_manager.AddHandler(transform_handler)

    
    custom_handler = CustomIntPublisher("custom_counter", 1)  
    ros_manager.AddHandler(custom_handler)

    
    ros_manager.Initialize()

    
    time_step = 0.01
    max_time = 10.0
    current_time = 0

    
    while current_time < max_time:
        
        ros_manager.Update(current_time)

        
        my_system.DoStepDynamics(time_step)

        
        current_time += time_step

        
        rospy.sleep(time_step)

    rospy.signal_shutdown("Simulation complete")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass