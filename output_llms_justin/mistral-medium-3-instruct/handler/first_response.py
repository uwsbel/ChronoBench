#!/usr/bin/env python3

import pychrono as chrono
import pychrono.ros as chrono_ros
import rospy
from std_msgs.msg import Int32

# Custom ROS handler to publish integer messages
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
    # Initialize ROS node
    rospy.init_node('chrono_ros_simulation')

    # Create a Chrono physical system
    my_system = chrono.ChSystemNSC()
    my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravitational acceleration

    # Set material properties
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(0.4)
    material.SetRestitution(0.2)

    # Create a fixed floor
    floor = chrono.ChBodyEasyBox(10, 0.5, 10, 1000, True, True, material)
    floor.SetPos(chrono.ChVectorD(0, -1, 0))
    floor.SetBodyFixed(True)
    my_system.Add(floor)

    # Create a movable box
    box = chrono.ChBodyEasyBox(1, 1, 1, 10, True, True, material)
    box.SetPos(chrono.ChVectorD(0, 2, 0))
    my_system.Add(box)

    # Create ROS manager
    ros_manager = chrono_ros.ChRosManager()

    # Add standard handlers
    clock_handler = chrono_ros.ChRosClockHandler(10)  # 10Hz publish rate
    ros_manager.AddHandler(clock_handler)

    body_handler = chrono_ros.ChRosBodyHandler(box, "box", 10)  # 10Hz publish rate
    ros_manager.AddHandler(body_handler)

    transform_handler = chrono_ros.ChRosTransformHandler(box, "box_transform", 10)  # 10Hz publish rate
    ros_manager.AddHandler(transform_handler)

    # Add custom handler
    custom_handler = CustomIntPublisher("custom_counter", 1)  # 1Hz publish rate
    ros_manager.AddHandler(custom_handler)

    # Initialize the ROS manager
    ros_manager.Initialize()

    # Simulation parameters
    time_step = 0.01
    max_time = 10.0
    current_time = 0

    # Simulation loop
    while current_time < max_time:
        # Update ROS communication
        ros_manager.Update(current_time)

        # Advance the simulation
        my_system.DoStepDynamics(time_step)

        # Update time
        current_time += time_step

        # Try to maintain real-time execution
        rospy.sleep(time_step)

    rospy.signal_shutdown("Simulation complete")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass