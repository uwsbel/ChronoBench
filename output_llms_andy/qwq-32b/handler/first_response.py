import pychrono as chrono
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

def main():
    # Initialize Chrono system
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

    # Create floor (static body)
    floor = chrono.ChBodyEasyBox(10, 1, 1, 1000, True, True)
    floor.SetPos(chrono.ChVectorD(0, 0, -1))
    floor.SetBodyFixed(True)
    system.Add(floor)

    # Create movable box
    box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True)
    box.SetPos(chrono.ChVectorD(0, 0, 2))
    system.Add(box)

    # Initialize ROS
    rclpy.init()
    node = Node('chrono_ros_node')
    publisher = node.create_publisher(Int32, 'test_topic', 10)

    # Custom ROS publisher handler
    class ROSCustomHandler(chrono.ChPhysicsItem):
        def __init__(self, pub):
            super().__init__()
            self.publisher = pub

        def Update(self):
            msg = Int32()
            msg.data = int(system.GetChTime() * 10)  # Example: publish time*10 as integer
            self.publisher.publish(msg)

    custom_handler = ROSCustomHandler(publisher)
    system.GetEventManager().AddSynchronizeStepEventPostFunction(custom_handler)

    # Setup ROS manager
    manager = chrono_ros.ChManager()
    manager.Add(chrono_ros.ChRtClock())  # Real-time clock handler
    manager.Add(chrono_ros.ChBodyHandler(system))  # Body handler
    manager.Add(chrono_ros.ChTransformPublisher())  # Transform handler
    manager.Add(custom_handler)  # Register custom handler

    # Simulation loop
    try:
        while rclpy.ok():
            time = system.GetChTime()
            manager.DoStepDynamics(time, 0.001)  # Step size 1ms
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()