import chrono as ch
import rospy
from rospy.topics import Publisher
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32

# Initialize PyChrono environment
ch.Initialize()

# Create a Chrono system with gravitational acceleration
system = ch.ChronoSystem()
system.SetGravitationalAcceleration(ch.ChVector3(0, -9.81, 0))

# Create a fixed floor
floor_body = ch.ChBodyEasyFloor(
    ch.ChVector3(0, -5, 0),  # position
    ch.ChVector3(10, 0.1, 10),  # size
    ch.ChColor(0.5, 0.5, 0.5)  # color
)
floor_body.SetBodyFixed(True)
system.AddBody(floor_body)

# Create a movable box
box_body = ch.ChBodyEasyBox(
    ch.ChVector3(0, 0, 0),  # position
    ch.ChVector3(1, 1, 1),  # size
    ch.ChColor(1, 0, 0)  # color
)
box_body.SetMass(10)
box_body.SetInertiaXX(ch.ChVector3(1, 1, 1))
system.AddBody(box_body)

# Custom ROS handler to publish integer messages
class CustomROSHandler:
    def __init__(self, topic_name):
        self.publisher = Publisher(topic_name, Int32, 10)

    def publish_message(self, message):
        msg = Int32()
        msg.data = message
        self.publisher.publish(msg)

# Initialize ROS node
rospy.init_node('chrono_ros_simulation')

# Create and configure ROS handlers
clock_handler = ch.ChROS_ClockHandler()
body_handler = ch.ChROS_BodyHandler(system)
transform_handler = ch.ChROS_TransformHandler(system)
custom_handler = CustomROSHandler('/chrono_simulation/message')

# Register handlers with ROS manager
ros_manager = ch.ChROS_Manager()
ros_manager.RegisterHandler(clock_handler)
ros_manager.RegisterHandler(body_handler)
ros_manager.RegisterHandler(transform_handler)
ros_manager.RegisterHandler(custom_handler)

# Simulation loop
time_step = 0.01
while not rospy.is_shutdown():
    # Advance the Chrono system
    system.DoStepDynamics(time_step)

    # Update ROS communication
    ros_manager.SpinOnce(time_step)

    # Publish a custom message
    custom_handler.publish_message(int(system.GetChTime()))

    # Maintain real-time execution
    rate = rospy.Rate(100)  # 100 Hz
    rate.sleep()