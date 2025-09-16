import chrono as ch
import rospy
from rospy.topics import Publisher
from std_msgs.msg import Int32

# Initialize PyChrono environment
ch.Initialize()

# Create a Chrono system with gravitational acceleration
system = ch.ChronoSystem()
system.SetGravitationalAcceleration(ch.ChVector3(0, -9.81, 0))

# Create a fixed floor
floor_body = ch.ChBodyEasyFloor("floor", ch.ChVector3(0, -10, 0))
floor_body.SetFixed(True)
system.Add(floor_body)

# Create a movable box
box_body = ch.ChBodyEasyBox(ch.ChVector3(1, 0, 0), ch.ChVector3(1, 1, 1), "box", ch.ChMaterialDesc().GetDefault())
box_body.SetPos(ch.ChVector3(0, 1, 0))
system.Add(box_body)

# Custom ROS handler to publish integer messages
class CustomROSHandler:
    def __init__(self, topic_name):
        self.publisher = Publisher(topic_name, Int32, 10)

    def publish_message(self, message):
        msg = Int32()
        msg.data = message
        self.publisher.publish(msg)

# ROS manager setup
class ChronoROSMgr:
    def __init__(self, system, topic_name):
        self.system = system
        self.topic_name = topic_name
        self.clock_handler = ch.ChROS_ClockHandler(system.GetChTimeStep())
        self.body_handler = ch.ChROS_BodyHandler(system)
        self.transform_handler = ch.ChROS_TransformHandler(system)
        self.custom_handler = CustomROSHandler(topic_name)

    def configure(self):
        self.clock_handler.Initialize()
        self.body_handler.Initialize()
        self.transform_handler.Initialize()

    def register_handlers(self):
        ch.ChROS_Manager.RegisterHandler(self.clock_handler)
        ch.ChROS_Manager.RegisterHandler(self.body_handler)
        ch.ChROS_Manager.RegisterHandler(self.transform_handler)
        ch.ChROS_Manager.RegisterHandler(self.custom_handler)

    def run(self):
        rate = rospy.Rate(1 / self.system.GetChTimeStep())
        while not rospy.is_shutdown():
            self.system.DoStepDynamics()
            self.clock_handler.Update()
            self.body_handler.Update()
            self.transform_handler.Update()
            self.custom_handler.publish_message(int(self.system.GetChTime()))
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node('chrono_ros_integration', anonymous=True)
    system.SetChTimeStep(0.01)

    chrono_ros_mgr = ChronoROSMgr(system, "/chrono_sim_time")
    chrono_ros_mgr.configure()
    chrono_ros_mgr.register_handlers()

    chrono_ros_mgr.run()