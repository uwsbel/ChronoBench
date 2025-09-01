import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import rospy
from std_msgs.msg import Int32

# Custom ROS handler to publish integer messages
class CustomROSHandler:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('chrono_ros_node', anonymous=True)
        self.publisher = rospy.Publisher('chrono_topic', Int32, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz

    def publish_message(self, message):
        self.publisher.publish(message)
        self.rate.sleep()

# Initialize the PyChrono system
chrono.SetChronoDataPath('/path/to/chrono/data/')  # Set your Chrono data path
system = chrono.ChSystemNSC()

# Set gravitational acceleration
gravity = chrono.ChVectorD(0, -9.81, 0)
system.Set_G_acc(gravity)

# Create a fixed floor
floor = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True)
floor.SetPos(chrono.ChVectorD(0, -0.05, 0))
floor.SetBodyFixed(True)
system.Add(floor)

# Create a movable box
box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True)
box.SetPos(chrono.ChVectorD(0, 0.5, 0))
system.Add(box)

# Set material properties for the box
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.4)
box.SetMaterialSurface(material)

# Initialize ROS handler
ros_handler = CustomROSHandler()

# Simulation loop
while not rospy.is_shutdown():
    # Advance the simulation
    time_step = 0.01  # 10 ms time step
    system.DoStepDynamics(time_step)

    # Publish the current box position as an integer (for example, the y-coordinate)
    box_position = box.GetPos()
    ros_handler.publish_message(int(box_position.y))

    # Optional: Print the current simulation time
    print(f"Simulation Time: {system.GetChTime():.2f} s")

# Clean up and shutdown
rospy.signal_shutdown("Simulation ended.")