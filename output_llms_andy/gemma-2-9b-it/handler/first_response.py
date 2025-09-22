import pychrono as chrono
import pychrono_ros  # Import the ROS integration library

# Initialize PyChrono environment
chrono.SetChronoData(chrono.ChSystemNSC)  # Use the NSC solver for faster simulations
system = chrono.ChSystemNSC()

# Set gravitational acceleration
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# Define physical material properties
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.5)
material.SetRestitution(0.2)

# Add a fixed floor
floor = chrono.ChBodyEasyBox(10, 10, 0.1, material)
floor.SetPos(chrono.ChVectorD(0, 0, -0.1))
floor.SetBodyFixed(True)
system.AddBody(floor)

# Add a movable box
box = chrono.ChBodyEasyBox(1, 1, 1, material)
box.SetPos(chrono.ChVectorD(0, 0, 1))
system.AddBody(box)

# ROS Initialization
ros = pychrono_ros.RosManager()
ros.Init()

# Custom ROS handler
class MyRosHandler(pychrono_ros.RosHandler):
    def __init__(self, topic):
        super().__init__()
        self.topic = topic

    def Update(self, system, step_size):
        # Publish integer messages based on box position
        position = box.GetPos()
        msg = {"x": int(position.x), "y": int(position.y), "z": int(position.z)}
        ros.Publish(self.topic, msg)

# Register ROS handlers
ros.RegisterBodyHandler(box)  # Register the box for position updates
ros.RegisterClockHandler()
ros.RegisterTransformHandler()
ros.RegisterRosHandler(MyRosHandler("/my_topic"))  # Register the custom handler

# Simulation loop
while True:
    system.DoStepDynamics(0.01)  # Advance the simulation by 0.01 seconds
    ros.Update()  # Update ROS communication