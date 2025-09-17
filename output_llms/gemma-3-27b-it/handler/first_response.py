import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chronoros
import time
import rospy
from std_msgs.msg import Int32

class CustomRosHandler(chronoros.ChRosHandler):
    """
    A custom ROS handler to publish integer messages.
    """
    def __init__(self, topic_name):
        super().__init__()
        self.topic_name = topic_name
        self.publisher = rospy.Publisher(self.topic_name, Int32, queue_size=10)

    def DoPublish(self):
        """
        Publish an integer message to the specified topic.
        """
        msg = Int32()
        msg.data = chrono.ChTime().GetSeconds()  # Example: Publish current simulation time
        self.publisher.publish(msg)
        rospy.loginfo(f"Published time: {msg.data} to topic: {self.topic_name}")


def main():
    # Initialize ROS node
    rospy.init_node('pychrono_ros_example', anonymous=True)

    # Initialize PyChrono
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

    # Create physical material properties
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(0.4)
    material.SetRestitution(0.2)

    # Create a fixed floor
    body_floor = chrono.ChBodyEasy()
    body_floor.SetBodyFixed(True)
    body_floor.SetPos(chrono.ChVectorD(0, -0.5, 0))
    body_floor.SetMaterialSurface(material)
    body_floor.AddBox(10, 0.2, 10)
    system.Add(body_floor)

    # Create a movable box
    body_box = chrono.ChBodyEasy()
    body_box.SetPos(chrono.ChVectorD(0, 1, 0))
    body_box.SetMaterialSurface(material)
    body_box.AddBox(0.5, 0.5, 0.5)
    system.Add(body_box)

    # Create a custom ROS handler
    ros_topic_name = "/chrono_time"
    custom_handler = CustomRosHandler(ros_topic_name)

    # Create a ROS manager
    ros_manager = chronoros.ChRosManager()
    ros_manager.SetSystem(system)
    ros_manager.SetClockHandler(True)
    ros_manager.SetBodyHandler(True)
    ros_manager.SetTransformHandler(True)
    ros_manager.RegisterHandler(custom_handler)

    # Initialize ROS communication
    ros_manager.Initialize()

    # Simulation loop
    time_step = 0.005
    while True:
        # Advance the simulation
        system.DoStepDynamics(time_step)

        # Update ROS communication
        ros_manager.Update()

        # Maintain real-time execution
        real_time = 0.005
        sleep_time = max(0, real_time - time.time() + ros_manager.GetLastTime())
        time.sleep(sleep_time)
        
        if rospy.is_shutdown():
            break

    # Clean up
    ros_manager.Shutdown()
    rospy.loginfo("Simulation finished.")

if __name__ == "__main__":
    main()