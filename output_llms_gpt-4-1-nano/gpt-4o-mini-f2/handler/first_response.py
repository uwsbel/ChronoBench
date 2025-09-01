import pychrono as chrono
import pychrono.ros as chros
import rclpy
from rclpy.node import Node

# Create a Chrono system
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Set gravity in the negative Z direction

# Define physical material properties
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.5)  # Set friction coefficient
mat.SetRestitution(0.01)  # Set restitution coefficient (bounciness)

# Create a floor object
floor = chrono.ChBodyEasyBox(10, 10, 1, 1000, True, True, mat)
floor.SetPos(chrono.ChVector3d(0, 0, -0.5))  # Position the floor slightly above the origin
floor.SetFixed(True)  # Fix the floor in place
system.Add(floor)  # Add the floor to the simulation system

# Create a box object
box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, mat)
box.SetPos(chrono.ChVector3d(0, 0, 2))  # Position the box above the floor
box.SetRot(chrono.QuatFromAngleZ(chrono.CH_PI / 4))  # Rotate the box 45 degrees around the Z-axis
system.Add(box)  # Add the box to the simulation system

# Create and configure the ROS manager
ros_manager = chros.ChROSPythonManager()
ros_manager.SetVerbose(True)  # Enable verbose output for the ROS manager
ros_manager.Initialize()  # Initialize the ROS manager

# Define a custom ROS handler to publish integer messages
class IntPublisher(chros.ChROSHandler):
    def __init__(self, topic):
        super().__init__()  # Initialize the base class
        self.topic = topic  # Set the topic for publishing
        self.publisher = None  # Initialize the publisher
        self.count = 0  # Initialize a counter

    def OnInit(self):
        # Create a ROS publisher for the specified topic
        self.publisher = self.GetNode().create_publisher(int, self.topic, 10)
        print(f"Publishing to topic: {self.topic}")

    def OnAdvance(self, time):
        # Publish an integer message to the topic
        msg = int(self.count)
        self.publisher.publish(msg)
        print(f"Published: {msg} to {self.topic}")
        self.count += 1  # Increment the counter

# Create and register the IntPublisher handler
int_pub_handler = IntPublisher("/chrono/test_int")
ros_manager.RegisterHandler(int_pub_handler)

# Configure the ROS manager
ros_manager.RegisterClockHandler(chros.ChROSPythonClockHandler())
ros_manager.RegisterBodyHandler(chros.ChROSPythonBodyHandler(box, "~/box"))
ros_manager.RegisterTFHandler(chros.ChROSPythonTFHandler(box, floor, "~/box_frame", "~/floor_frame"))

# Start the simulation loop
time = 0
time_step = 1e-3  # Define the simulation time step
time_end = 10  # Set the duration of the simulation

while time < time_end:
    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)
    time = system.GetChTime()  # Update the simulation time

    # Update ROS communication
    if not ros_manager.Update(time, time_step):
        break  # Exit the loop if the ROS manager update fails

# Shutdown the ROS manager after the simulation ends
ros_manager.Shutdown()