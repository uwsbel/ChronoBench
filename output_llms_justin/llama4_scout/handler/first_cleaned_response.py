import chrono as ch
import rospy
from rospy.topics import Publisher
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32


ch.Initialize()


system = ch.ChronoSystem()
system.SetGravitationalAcceleration(ch.ChVector3(0, -9.81, 0))


floor_body = ch.ChBodyEasyFloor(
    ch.ChVector3(0, -5, 0),  
    ch.ChVector3(10, 0.1, 10),  
    ch.ChColor(0.5, 0.5, 0.5)  
)
floor_body.SetBodyFixed(True)
system.AddBody(floor_body)


box_body = ch.ChBodyEasyBox(
    ch.ChVector3(0, 0, 0),  
    ch.ChVector3(1, 1, 1),  
    ch.ChColor(1, 0, 0)  
)
box_body.SetMass(10)
box_body.SetInertiaXX(ch.ChVector3(1, 1, 1))
system.AddBody(box_body)


class CustomROSHandler:
    def __init__(self, topic_name):
        self.publisher = Publisher(topic_name, Int32, 10)

    def publish_message(self, message):
        msg = Int32()
        msg.data = message
        self.publisher.publish(msg)


rospy.init_node('chrono_ros_simulation')


clock_handler = ch.ChROS_ClockHandler()
body_handler = ch.ChROS_BodyHandler(system)
transform_handler = ch.ChROS_TransformHandler(system)
custom_handler = CustomROSHandler('/chrono_simulation/message')


ros_manager = ch.ChROS_Manager()
ros_manager.RegisterHandler(clock_handler)
ros_manager.RegisterHandler(body_handler)
ros_manager.RegisterHandler(transform_handler)
ros_manager.RegisterHandler(custom_handler)


time_step = 0.01
while not rospy.is_shutdown():
    
    system.DoStepDynamics(time_step)

    
    ros_manager.SpinOnce(time_step)

    
    custom_handler.publish_message(int(system.GetChTime()))

    
    rate = rospy.Rate(100)  
    rate.sleep()