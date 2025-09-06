import pychrono as chrono
import pychrono.ros as chrono_ros
import rospy
from std_msgs.msg import Int32

# Initialize ROS node
rospy.init_node('chrono_ros_simulation', anonymous=True)

# Create a Chrono physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravitational acceleration

# Set material properties
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.4)
material.SetRestitution(0.2)

# Create a fixed floor
floor_body = chrono.ChBody()
floor_body.SetPos(chrono.ChVectorD(0, -1, 0))
floor_body.SetBodyFixed(True)
floor_body.GetVisualModel().AddBox(10, 1, 10, True, chrono.ChColor(0.5, 0.5, 0.5))
floor_body.GetCollisionModel().AddBox(10, 1, 10)
floor_body.GetCollisionModel().SetMaterialSurface(material)
system.Add(floor_body)

# Create a movable box
box_body = chrono.ChBody()
box_body.SetPos(chrono.ChVectorD(0, 2, 0))
box_body.SetMass(1.0)
box_body.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
box_body.GetVisualModel().AddBox(1, 1, 1, True, chrono.ChColor(0.8, 0.2, 0.2))
box_body.GetCollisionModel().AddBox(1, 1, 1)
box_body.GetCollisionModel().SetMaterialSurface(material)
system.Add(box_body)

# Custom ROS handler to publish integer messages
class CustomIntPublisher(chrono_ros.ChRosHandler):
    def __init__(self, topic_name):
        super().__init__()
        self.pub = rospy.Publisher(topic_name, Int32, queue_size=10)
        self.counter = 0

    def Update(self):
        msg = Int32()
        msg.data = self.counter
        self.pub.publish(msg)
        self.counter += 1

# Create ROS manager
ros_manager = chrono_ros.ChRosManager()

# Add standard handlers
ros_manager.AddHandler(chrono_ros.ChRosClockHandler())
ros_manager.AddHandler(chrono_ros.ChRosBodyHandler())
ros_manager.AddHandler(chrono_ros.ChRosTransformHandler())

# Add our custom handler
custom_handler = CustomIntPublisher('/chrono_counter')
ros_manager.AddHandler(custom_handler)

# Set up visualization (optional)
if True:
    from pychrono import irrlicht as chrono_irr
    vis = chrono_irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(800, 600)
    vis.SetWindowTitle('PyChrono ROS Simulation')
    vis.Initialize()
    vis.AddCamera(chrono.ChVectorD(3, 2, 0))
    vis.AddTypicalLogo()
    vis.AddTypicalSky()
    vis.AddLightWithShadow(chrono.ChVectorD(3, 5, 0), chrono.ChVectorD(0, 0, 0), 3, 1, 5, 30, 512)

# Simulation parameters
time_step = 0.01
simulation_time = 10.0
current_time = 0.0

# Main simulation loop
while current_time < simulation_time:
    # Advance simulation
    system.DoStepDynamics(time_step)

    # Update ROS communication
    ros_manager.Update()

    # Update visualization if enabled
    if 'vis' in locals():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Maintain real-time execution
    rospy.sleep(time_step)

    current_time += time_step

# Clean up
ros_manager.Shutdown()
if 'vis' in locals():
    vis.CloseDevice()