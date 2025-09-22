import pychrono.core as chrono
import pychrono.ros as ros
import rospy
from std_msgs.msg import Int32


rospy.init_node('chrono_ros_simulation')


chrono_system = chrono.ChSystemNSC()
chrono_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


floor = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True)
floor.SetPos(chrono.ChVectorD(0, -0.05, 0))
floor.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
chrono_system.Add(floor)


box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True)
box.SetPos(chrono.ChVectorD(0, 0.5, 0))
box.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
chrono_system.Add(box)


pub = rospy.Publisher('chrono_integer', Int32, queue_size=10)


def ros_handler():
    rate = rospy.Rate(10)  
    while not rospy.is_shutdown():
        chrono_system.DoStepDynamics(0.01)  

        
        msg = Int32()
        msg.data = int(chrono_system.GetChTime() * 100)  
        pub.publish(msg)

        rate.sleep()  

if __name__ == "__main__":
    try:
        ros_handler()
    except rospy.ROSInterruptException:
        pass