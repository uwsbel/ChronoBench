import pychrono as ch
import pychrono.ros as chros
import pychrono.irrlicht as chrirr  # Added Irrlicht module

import rclpy.publisher
from std_msgs.msg import Int64

# Define a custom ROS handler for publishing integer messages.
class MyCustomHandler(chros.ChROSHandler):
    """This custom handler will publish integer messages to a specified ROS topic."""

    def __init__(self, rate, topic):  # Modified to accept rate
        super().__init__(rate)  # Use specified rate instead of 1Hz

        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.ticker = 0  # Initialize a counter for published messages.

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        """Initialize the ROS publisher."""
        print(f"Creating publisher for topic {self.topic} ...")
        # Create a ROS publisher for the specified topic.
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 1)
        return True  # Return True to indicate successful initialization.

    def Tick(self, time: float):
        """Publish an integer message to the ROS topic."""
        print(f"Publishing {self.ticker} ...")
        msg = Int64()  # Create a message object of type Int64.
        msg.data = self.ticker  # Set the message data to the current ticker value.
        self.publisher.publish(msg)  # Publish the message to the ROS topic.
        self.ticker += 1  # Increment the ticker for the next message.

def main():
    # Create the Chrono simulation system.
    sys = ch.ChSystemNSC()
    sys.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.81))  # Set gravitational acceleration.

    # Define physical material properties for contact.
    phys_mat = ch.ChContactMaterialNSC()
    phys_mat.SetFriction(0.5)  # Set friction coefficient.

    # Create a floor object.
    floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
    floor.SetPos(ch.ChVector3d(0, 0, -1))  # Position the floor.
    floor.SetFixed(True)  # Fix the floor in place.
    floor.SetName("base_link")  # Set the name for ROS communication.
    # Apply texture to floor
    floor_texture = ch.GetChronoDataFile("textures/concrete.jpg")
    floor.GetVisualShape(0).SetTexture(floor_texture)
    sys.Add(floor)  # Add the floor to the simulation system.

    # Create a box object.
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 5))  # Position the box above the floor.
    box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))  # Rotate the box slightly.
    box.SetName("box")  # Set the name for ROS communication.
    # Apply texture to box
    box_texture = ch.GetChronoDataFile("textures/cubetexture_wood.png")
    box.GetVisualShape(0).SetTexture(box_texture)
    sys.Add(box)  # Add the box to the simulation system.

    # Create Irrlicht visualization system
    vis = chrirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("PyChrono ROS Simulation")
    vis.Initialize()
    vis.AddLogo(ch.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(ch.ChVector3d(3, 3, 3), ch.ChVector3d(0, 0, 0))
    vis.AddTypicalLights()

    # Create and configure the ROS manager.
    ros_manager = chros.ChROSPythonManager()
    publish_rate = 10  # Set publish rate to 10Hz
    
    # Register handlers with specified publish rate
    ros_manager.RegisterHandler(chros.ChROSClockHandler(publish_rate))
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(publish_rate, box, "~/box"))
    
    # Create and register transform handler
    tf_handler = chros.ChROSTFHandler(publish_rate)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)
    
    # Register custom handler with publish rate
    custom_handler = MyCustomHandler(publish_rate, "~/my_topic")
    ros_manager.RegisterPythonHandler(custom_handler)

    # Initialize the ROS manager.
    ros_manager.Initialize()

    # Simulation control variables
    time = 0
    time_step = 1e-3  # Simulation time step
    time_end = 30     # Simulation duration
    step_number = 0   # Step counter
    render_steps = 20 # Render every 20 simulation steps

    realtime_timer = ch.ChRealtimeStepTimer()  # Create a real-time step timer.
    while time < time_end:
        # Render only every 'render_steps' steps
        if step_number % render_steps == 0:
            if not vis.Run():
                break
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        sys.DoStepDynamics(time_step)  # Advance simulation
        time = sys.GetChTime()         # Update simulation time
        step_number += 1               # Increment step counter

        # Update ROS (removed incorrect condition check)
        ros_manager.Update(time, time_step)

        realtime_timer.Spin(time_step)  # Maintain real-time execution

# Entry point of the script.
if __name__ == "__main__":
    main()