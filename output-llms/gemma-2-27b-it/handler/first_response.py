import pychrono as chrono
import pychrono.ros as chroro
import rospy

# Initialize PyChrono environment
my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Material properties
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.5)
material.SetRestitution(0.2)

# Create a fixed floor
floor = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True, material)
floor.SetPos(chrono.ChVectorD(0, -0.1, 0))
my_system.Add(floor)

# Create a movable box
box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 10, True, True, material)
box.SetPos(chrono.ChVectorD(0, 1, 0))
my_system.Add(box)

# Initialize ROS
rospy.init_node('chrono_ros_node')

# Create ROS manager
ros_manager = chroro.ChROSManager(my_system)

# Configure ROS handlers
ros_manager.SetClockHandler(chroro.ChROSClockHandler())
ros_manager.SetBodyHandler(chroro.ChROSBodyHandler())
ros_manager.SetTransformHandler(chroro.ChROSTransformHandler())

# Custom ROS handler for publishing integer messages
class CustomROSHandler(chroro.ChROSHandler):
    def __init__(self, topic_name):
        super().__init__()
        self.topic_name = topic_name
        self.publisher = rospy.Publisher(self.topic_name, Int32, queue_size=10)

    def OnUpdate(self, data):
        message = Int32()
        message.data = int(data)  # Convert data to integer
        self.publisher.publish(message)

# Register custom handler
custom_handler = CustomROSHandler('/chrono_data')
ros_manager.RegisterHandler(custom_handler)

# Simulation loop
while not rospy.is_shutdown():
    # Advance the Chrono system
    my_system.DoStepDynamics(0.01)

    # Update ROS communication
    ros_manager.Update()

    # Publish custom data (e.g., box position)
    custom_handler.OnUpdate(box.GetPos().y)

    # Maintain real-time execution
    rospy.rostime.wallsleep(0.01)