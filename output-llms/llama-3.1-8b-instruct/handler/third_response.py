"""
Import necessary modules.
"""
import pychrono as ch
import pychrono.ros as chros

import rclpy.publisher
from std_msgs.msg import Int64

"""
Define a custom ROS handler for publishing integer messages.
"""
class MyCustomHandler(chros.ChROSHandler):
    """This custom handler will publish integer messages to a specified ROS topic."""

    def __init__(self, topic, publish_rate):
        super().__init__(publish_rate)  # Initialize the handler with a specified publishing rate.

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
    floor.SetTexture(ch.ChTexture2D("floor_texture.png"))  # Set the texture for the floor.
    sys.Add(floor)  # Add the floor to the simulation system.

    # Create a box object.
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 5))  # Position the box above the floor.
    box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))  # Rotate the box slightly.
    box.SetName("box")  # Set the name for ROS communication.
    box.SetTexture(ch.ChTexture2D("box_texture.png"))  # Set the texture for the box.
    sys.Add(box)  # Add the box to the simulation system.

    # Create and configure the ROS manager.
    ros_manager = chros.ChROSPythonManager()
    
    # Register a clock handler for the simulation time.
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    
    # Register a body handler to communicate the box's state.
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/box"))
    
    # Create and register a transform handler for coordinate transformations.
    tf_handler = chros.ChROSTFHandler(30)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)
    
    # Register the custom handler to publish messages.
    custom_handler = MyCustomHandler("~/my_topic", 10)  # Set the publish rate to 10 Hz.
    ros_manager.RegisterPythonHandler(custom_handler)

    # Initialize the ROS manager.
    ros_manager.Initialize()

    # Set up visualization with Irrlicht.
    irr = ch.ChIrrApp(sys, "ChronoPyROS", ch.EAAD_NO_WINDOW)
    irr.AddSkySphere(True, 1000)
    irr.AddLight(ch.ChVector3d(0, 0, 10), ch.ChVector3d(0, 0, 0), 1000, 0.5)
    irr.AddLight(ch.ChVector3d(0, 0, -10), ch.ChVector3d(0, 0, 0), 1000, 0.5)
    irr.SetCurrentCameraTarget(ch.ChVector3d(0, 0, 0))
    irr.SetCurrentCameraPosition(ch.ChVector3d(0, 0, 10))
    irr.SetRenderWindowFlags(ch.EAWF_FULLSCREEN)
    irr.SetWindowSize(800, 600)
    irr.SetWindowFlags(ch.EAWF_FULLSCREEN)

    # Set variables for controlling rendering frame rate.
    step_number = 0
    render_step_size = 10
    render_steps = 100

    # Run the simulation loop.
    time = 0
    time_step = 1e-3  # Define the simulation time step.
    time_end = 30  # Set the duration for the simulation.

    realtime_timer = ch.ChRealtimeStepTimer()  # Create a real-time step timer.
    while time < time_end:
        sys.DoStepDynamics(time_step)  # Advance the simulation by one time step.
        time = sys.GetChTime()  # Update the simulation time.

        if not ros_manager.Update(time, time_step):  # Update ROS communication.
            break  # Exit the loop if the ROS manager indicates a problem.

        step_number += 1
        if step_number % render_step_size == 0:
            # Update the scene every few steps.
            irr.BeginScene(True, True, ch.ChColor(0.2, 0.2, 0.2))
            irr.DrawAll()
            irr.EndScene()
            step_number = 0

        realtime_timer.Spin(time_step)  # Maintain real-time step execution.

# Entry point of the script.
if __name__ == "__main__":
    main()