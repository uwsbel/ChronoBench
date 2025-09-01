import pychrono as chrono
import pychrono.robot as robot
import rospy
import tf


chrono.SetChronoDataPath("/path/to/chrono/data")
chrono.SetChronoDataPath("/path/to/chrono/data")


rospy.init_node("pychrono_ros_node")


my_system = chrono.ChSystemNSC()


my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))