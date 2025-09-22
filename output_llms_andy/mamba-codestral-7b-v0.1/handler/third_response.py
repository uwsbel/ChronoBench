import pychrono as ch
import pychrono.irrlicht as chirr

import rclpy.publisher
from std_msgs.msg import Int64

# Define a custom ROS handler for publishing integer messages.
class MyCustomHandler(chirr.ChIrrlichtHandler):
    """This custom handler will publish integer messages to a specified ROS topic."""

    def __init__(self, topic, publish_rate):
        super().__init__(1)  # Initialize the handler with a 1 Hz publishing rate.

        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.ticker = 0  # Initialize a counter for published messages.
        self.publish_rate = publish_rate

    def Initialize(self, interface: chirr.ChIrrlichtInterface) -> bool:
        """Initialize the ROS publisher."""
        print(f"Creating publisher for topic {self.topic} ...")
        # Create a ROS publisher for the specified topic.
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 1)
        return True  # Return True to indicate successful initialization.

    def Tick(self, time: float):
        """Publish an integer message to the ROS topic."""
        if self.ticker % self.publish_rate == 0:
            print(f"Publishing {self.ticker} ...")
            msg = Int64()  # Create a message object of type Int64.
            msg.data = self.ticker  # Set the message data to the current ticker value.
            self.publisher.publish(msg)  # Publish the message to the ROS topic.
        self.ticker += 1  # Increment the ticker for the next message.

def main():
    # Create the Chrono simulation system.
    sys = ch.ChSystemNSC()
    sys.SetGravitationalAcceleration(ch.ChVector3D(0, 0, -9.81))  # Set gravitational acceleration.

    # Define physical material properties for contact.
    phys_mat = ch.ChContactMaterialNSC()
    phys_mat.SetFriction(0.5)  # Set friction coefficient.

    # Create a floor object.
    floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
    floor.SetPos(ch.ChVector3D(0, 0, -1))  # Position the floor.
    floor.SetFixed(True)  # Fix the floor in place.
    floor.SetName("base_link")  # Set the name for ROS communication.
    sys.Add(floor)  # Add the floor to the simulation system.

    # Create a box object.
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3D(0, 0, 5))  # Position the box above the floor.
    box.SetRot(ch.Q_from_AngAxis(.2, ch.ChVector3D(1, 0, 0)))  # Rotate the box slightly.
    box.SetName("box")  # Set the name for ROS communication.
    sys.Add(box)  # Add the box to the simulation system.

    # Create and configure the ROS manager.
    ros_manager = chirr.ChIrrlichtManager()

    # Register a clock handler for the simulation time.
    ros_manager.RegisterHandler(chirr.ChIrrlichtClockHandler())

    # Register a body handler to communicate the box's state.
    ros_manager.RegisterHandler(chirr.ChIrrlichtBodyHandler(25, box, "~/box", 10))

    # Create and register a transform handler for coordinate transformations.
    tf_handler = chirr.ChIrrlichtTFHandler(30)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)

    # Register the custom handler to publish messages.
    custom_handler = MyCustomHandler("~/my_topic", 10)
    ros_manager.RegisterPythonHandler(custom_handler)

    # Initialize the ROS manager.
    ros_manager.Initialize()

    # Set up Irrlicht visualization.
    vis = chirr.ChIrrlichtVisualization()
    vis.SetWindowSize(800, 600)
    vis.SetWindowTitle('PyChrono Irrlicht Visualization')
    vis.SetCameraPosition(ch.ChVector3D(0, 0, 10), ch.ChVector3D(0, 0, 0), ch.ChVector3D(0, 1, 0))
    vis.AddLogo()
    vis.AddSkyBox()
    vis.AddTypicalLights(ch.ChVector3D(-10, -10, 10), ch.ChVector3D(10, 10, 10), 250, 250, 250)

    # Set textures for the floor and box.
    floor.SetTexture(ch.ChTexture(ch.GetChronoDataFile('concrete.png')))
    box.SetTexture(ch.ChTexture(ch.GetChronoDataFile('concrete.png')))

    # Run the simulation loop.
    time = 0
    time_step = 1e-3  # Define the simulation time step.
    time_end = 30  # Set the duration for the simulation.

    realtime_timer = ch.ChRealtimeStepTimer()  # Create a real-time step timer.
    step_number = 0
    render_step_size = 10  # Number of steps between rendering frames.
    render_steps = 0  # Counter for rendering frames.
    while time < time_end:
        sys.DoStepDynamics(time_step)  # Advance the simulation by one time step.
        time = sys.GetChTime()  # Update the simulation time.

        if not ros_manager.Update(time, time_step):  # Update ROS communication.
            break  # Exit the loop if the ROS manager indicates a problem.

        realtime_timer.Spin(time_step)  # Maintain real-time step execution.

        if step_number % render_step_size == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            render_steps += 1

        step_number += 1

    print(f"Total rendering steps: {render_steps}")

# Entry point of the script.
if __name__ == "__main__":
    main()