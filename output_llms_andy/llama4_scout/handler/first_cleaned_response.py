import os
import rospy
from pyrobohusb import ROBOT
from chrono import *
from chrono_ros import *


ChronoEngine.Initialize()


system = ChSystem()
system.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81))


floor_body = ChBodyEasyFloor()
floor_body.SetPos(ChVector3d(0, 0, -5))
system.Add(floor_body)


box_body = ChBodyEasyBox(1, 1, 1, 1000)
box_body.SetPos(ChVector3d(0, 5, 0))
box_body.SetPos_dt(ChVector3d(0, 0, 0))
system.Add(box_body)


class CustomROSHandler(ROSHandler):
    def __init__(self, topic_name):
        super(CustomROSHandler, self).__init__()
        self.publisher = rospy.Publisher(topic_name, Int32, queue_size=10)

    def OnPostStep(self, step):
        msg = Int32()
        msg.data = int(step * 1000)  
        self.publisher.publish(msg)


rospy.init_node('chrono_ros_integration')


ros_manager = ROSManager()
clock_handler = ROSClockHandler()
ros_manager.RegisterHandler(clock_handler)

body_handler = ROSBodyHandler(system)
ros_manager.RegisterHandler(body_handler)

transform_handler = ROSBodyTransformHandler(system)
ros_manager.RegisterHandler(transform_handler)

custom_handler = CustomROSHandler('/chrono_simulation_step')
ros_manager.RegisterHandler(custom_handler)


real_time_factor = 1.0  
step_size = 0.01


while not rospy.is_shutdown():
    system.DoStepDynamics(step_size)
    ros_manager.SpinOnce(step_size * 1000)
    rate = rospy.Rate(1 / (step_size * real_time_factor))
    rate.sleep()


ChronoEngine.Terminate()