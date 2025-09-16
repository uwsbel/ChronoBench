import pychrono as chrono
import pychrono_ros as chrono_ros
import rospy
from std_msgs.msg import Int32

class CustomIntPublisherHandler(chrono_ros.ChRosHandler):
    def __init__(self, topic_name):
        super().__init__()
        self.publisher = rospy.Publisher(topic_name, Int32, queue_size=10)

    def Update(self):
        msg = Int32()
        msg.data = 42  # Example value, can be modified dynamically
        self.publisher.publish(msg)

def main():
    rospy.init_node('chrono_ros_simulation')

    # Initialize Chrono system with gravity
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

    # Create fixed floor
    floor = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True)
    floor.SetBodyFixed(True)
    floor.SetPos(chrono.ChVectorD(0, -1, 0))
    system.Add(floor)

    # Create movable box
    box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True)
    box.SetPos(chrono.ChVectorD(0, 2, 0))
    system.Add(box)

    # Initialize ROS handlers
    custom_handler = CustomIntPublisherHandler('/integer_topic')

    # Configure ROS manager
    manager = chrono_ros.ChManager()
    manager.SetSystem(system)

    # Add required handlers
    clock_handler = chrono_ros.ChRosClockHandler()
    clock_handler.SetRealtimeFactor(1.0)  # Real-time simulation
    manager.AddHandler(clock_handler)

    manager.AddHandler(chrono_ros.ChRosBodyLoader())  # Body loader
    manager.AddHandler(chrono_ros.ChRosTransformPublisher())  # TF publisher
    manager.AddHandler(custom_handler)  # Custom integer publisher

    # Simulation loop parameters
    step_size = 0.001  # Time step

    while not rospy.is_shutdown():
        # Synchronize with ROS time and advance simulation
        manager.Synchronize(chrono.GetSystemTimer().GetTime())
        system.DoStepDynamics(step_size)

if __name__ == '__main__':
    main()