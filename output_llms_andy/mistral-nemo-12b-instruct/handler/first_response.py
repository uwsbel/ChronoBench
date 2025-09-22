import chrono.chrono as c
import chrono.ros as cr
import rospy
import std_msgs.msg

# Initialize PyChrono environment
c.Initialize()

# Create Chrono system with gravitational acceleration
sys = c.ChSystemSMC()
sys.Set_Gravity(c.ChVectorD(0, -9.81, 0))

# Set physical material properties
mat_floor = c.ChMaterialSurfaceNSC()
mat_floor.SetFriction(0.6)
mat_box = c.ChMaterialSurfaceNSC()
mat_box.SetFriction(0.6)

# Add fixed floor
floor = c.ChBodyEasyBox(10, 0.1, 10, mat_floor, True, True)
floor.SetPos(c.ChVectorD(0, -0.5, 0))
floor.SetName("floor")
sys.Add(floor)

# Add movable box
box = c.ChBodyEasyBox(0.5, 0.5, 0.5, mat_box, True, True)
box.SetPos(c.ChVectorD(0, 0.5, 0))
box.SetName("box")
sys.Add(box)

# Set default parameters
box.SetBodyFixed(False)
box.SetPos(c.ChVectorD(0, 0.5, 0))
box.SetVel(c.ChVectorD(0, 0, 0))

# Initialize ROS node and manager
rospy.init_node('pychrono_ros_node')
ros_manager = cr.ChRosManager(sys)

# Create custom ROS handler to publish integer messages
class CustomRosHandler(cr.ChRosHandler):
    def __init__(self, name, topic):
        super(CustomRosHandler, self).__init__(name, topic)
        self.pub = rospy.Publisher(self.topic, std_msgs.msg.Int32, queue_size=10)

    def Update(self, my_frame):
        msg = std_msgs.msg.Int32()
        msg.data = int(my_frame.GetPos().y)
        self.pub.publish(msg)

# Configure ROS manager with clock, body, and transform handlers
clock_handler = cr.ChRosClockHandler()
body_handler = cr.ChRosBodyHandler()
transform_handler = cr.ChRosTransformHandler()
custom_handler = CustomRosHandler("box_pos_pub", "/box_pos_topic")

ros_manager.RegisterClockHandler(clock_handler)
ros_manager.RegisterBodyHandler(body_handler, "floor", "box")
ros_manager.RegisterTransformHandler(transform_handler, "floor", "box")
ros_manager.RegisterCustomHandler(custom_handler)

# Run simulation loop
while not rospy.is_shutdown():
    sys.DoStepDynamics(0.01)
    ros_manager.Synchronize()