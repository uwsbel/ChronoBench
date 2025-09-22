import pychrono.core as chrono
import pychrono.ros as ros
import rospy
from std_msgs.msg import Int32

# Initialize the ROS node
rospy.init_node('chrono_ros_simulation')

# Create a Chrono system
chrono_system = chrono.ChSystemNSC()
chrono_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravitational acceleration

# Create a fixed floor
floor = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True)
floor.SetPos(chrono.ChVectorD(0, -0.05, 0))
floor.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
chrono_system.Add(floor)

# Create a movable box
box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True)
box.SetPos(chrono.ChVectorD(0, 0.5, 0))
box.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
chrono_system.Add(box)

# Set up ROS publisher
pub = rospy.Publisher('chrono_integer', Int32, queue_size=10)

# Custom ROS handler
def ros_handler():
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        chrono_system.DoStepDynamics(0.01)  # Advance the Chrono simulation by 0.01 seconds

        # Publish an integer message (for example, the current time step)
        msg = Int32()
        msg.data = int(chrono_system.GetChTime() * 100)  # Example: convert time to an integer
        pub.publish(msg)

        rate.sleep()  # Sleep to maintain the desired rate

if __name__ == "__main__":
    try:
        ros_handler()
    except rospy.ROSInterruptException:
        pass