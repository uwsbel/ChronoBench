import pychrono as chrono
import pychrono.ros as ros
import rospy
from std_msgs.msg import Int32

# Initialize the Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravitational acceleration

# Set up material properties
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.3)
material.SetRestitution(0.4)

# Create a fixed floor
floor_body = chrono.ChBody()
floor_body.SetBodyFixed(True)
floor_body.SetCollide(True)
floor_body.GetCollisionModel().AddBox(10, 0.5, 10)  # Large floor
floor_body.GetCollisionModel().BuildModel()
floor_body.GetCollisionModel().SetMaterialSurface(material)
floor_body.SetPos(chrono.ChVectorD(0, -1, 0))
system.Add(floor_body)

# Create a movable box
box_body = chrono.ChBody()
box_body.SetMass(1.0)
box_body.SetInertiaXX(chrono.ChVectorD(1.0, 1.0, 1.0))
box_body.SetPos(chrono.ChVectorD(0, 2, 0))
box_body.SetCollide(True)
box_body.GetCollisionModel().AddBox(0.5, 0.5, 0.5)
box_body.GetCollisionModel().BuildModel()
box_body.GetCollisionModel().SetMaterialSurface(material)
system.Add(box_body)

# Initialize ROS node
rospy.init_node('chrono_ros_simulation', anonymous=True)

# Custom ROS handler to publish integer messages
class CustomIntPublisher(ros.ChRosHandler):
    def __init__(self, topic_name, queue_size=10):
        super(CustomIntPublisher, self).__init__()
        self.publisher = rospy.Publisher(topic_name, Int32, queue_size=queue_size)
        self.counter = 0

    def update(self):
        msg = Int32()
        msg.data = self.counter
        self.publisher.publish(msg)
        self.counter += 1

# Set up ROS manager
ros_manager = ros.ChRosManager()
ros_manager.SetClockHandler(ros.ChRosClockHandler())
ros_manager.AddBodyHandler(box_body, ros.ChRosBodyHandler())
ros_manager.AddTransformHandler(box_body, ros.ChRosTransformHandler())

# Add custom handler to ROS manager
custom_publisher = CustomIntPublisher('/chrono_sim_counter', queue_size=10)
ros_manager.AddCustomHandler(custom_publisher)

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

    # Increment time
    current_time += time_step

    # Maintain real-time execution
    ros_manager.SpinOnce(1.0 / 60.0)  # 60 Hz update rate

print("Simulation completed.")