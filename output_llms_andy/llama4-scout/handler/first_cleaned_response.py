import os
import rospy
from rospy.topics import Publisher
from chrono import chrono
from chrono.irrlicht import irrlicht
from chrono.pyomegaconst import *
from chrono_ros import chrono_ros


chrono.ChronoEngine.Initialize()


rospy.init_node('chrono_ros_integration')


system = chrono.ChronoSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3(0, -9.81, 0))


floor_body = chrono.ChBodyEasyFloor(chrono.ChCoordsys(chrono.ChVector3(0, -10, 0)))
floor_body.SetPosDt(chrono.ChVector3(0, 0, 0))
system.Add(floor_body)


box_body = chrono.ChBodyEasyBox(1, 1, 1, 1000)
box_body.SetPosDt(chrono.ChVector3(0, 0, 5))
box_body.SetRotDt(chrono.ChQuaternion(1, 0, 0, 0))
system.Add(box_body)


class CustomROSHandler(chrono_ros.ChronoROSHandler):
    def __init__(self, topic_name, publish_frequency):
        super(CustomROSHandler, self).__init__()
        self.publisher = Publisher(topic_name, rospy.msg.AnyMsg, 10)
        self.publish_frequency = publish_frequency

    def OnInitialize(self):
        pass

    def OnUpdate(self, step):
        msg = rospy.msg.Int32()
        msg.data = int(chrono.ChTime())
        self.publisher.publish(msg)


ros_manager = chrono_ros.ChronoROSMgr(chrono.ChTime())


ros_manager.RegisterHandler(chrono_ros.ChronoROS_ClockHandler(ros_manager, 0.01))


ros_manager.RegisterHandler(chrono_ros.ChronoROS_BodyHandler(ros_manager, box_body, "box_body"))


ros_manager.RegisterHandler(chrono_ros.ChronoROS_TransformHandler(ros_manager, box_body, "box_body_tf"))


custom_handler = CustomROSHandler("/chrono_sim/time", 10.0)
ros_manager.RegisterHandler(custom_handler)


while not rospy.is_shutdown():
    
    system.DoStepDynamics(0.01)

    
    ros_manager.SpinOnce(0.01)

    
    rate = rospy.Rate(100)
    rate.sleep()