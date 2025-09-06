import pychrono as chrono
import pychrono_ros as chrono_ros
import rospy
from std_msgs.msg import Int32

# Initialize Chrono system with gravity
my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create fixed floor
floor = chrono.ChBody()
floor.SetBodyFixed(True)
floor.SetPos(chrono.ChVectorD(0, -5, 0))
floor_shape = chrono.ChVisualShapeBox(chrono.ChVectorD(10, 1, 10))
floor.AddVisualShape(floor_shape)
my_system.Add(floor)

# Create movable box
box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True)
box.SetPos(chrono.ChVectorD(0, 0, 0))
my_system.Add(box)

# Initialize ROS manager
ros_manager = chrono_ros.ChROSManager()
ros_manager.Initialize()

# Add default ROS handlers
ros_manager.AddHandler(chrono_ros.ChROSClockHandler())  # Publish simulation time
ros_manager.AddHandler(chrono_ros.ChROSBodyHandler(my_system, box, "box_state"))  # Box state
ros_manager.AddHandler(chrono_ros.ChROSTransformHandler())  # Coordinate transforms

# Custom ROS handler for integer messages
class IntPublisherHandler(chrono_ros.ChROSHandler):
    def __init__(self):
        super().__init__("int_publisher")
        self.publisher = None
        self.count = 0

    def OnInitialize(self):
        self.publisher = self.node_handle.advertise("chrono_int", Int32, queue_size=10)

    def OnUpdate(self, time):
        if self.publisher:
            msg = Int32()
            msg.data = self.count
            self.publisher.publish(msg)
            self.count += 1

# Register custom handler
ros_manager.AddHandler(IntPublisherHandler())

# Simulation loop with real-time execution
time_step = 0.01
while ros_manager.Ok():
    my_system.DoStepDynamics(time_step)
    ros_manager.Update()