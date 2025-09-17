import pychrono as ch
import pychrono.ros as chros
import rclpy.publisher
from std_msgs.msg import Int64

# Define a custom ROS handler for publishing integer messages.
class MyCustomHandler(chros.ChROSHandler):
    """This custom handler will publish integer messages to a specified ROS topic."""

    def __init__(self, topic):
        super().__init__(10)  # Initialize the handler with a 10 Hz publishing rate.
        self.topic = topic
        self.publisher = None
        self.ticker = 0  # Initialize a counter for published messages.

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        """Initialize the ROS publisher."""
        print(f"Creating publisher for topic {self.topic} ...")
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 10)  # Update publishing rate to 10 Hz
        return True  # Return True to indicate successful initialization.

    def Tick(self, time: float):
        """Publish an integer message to the ROS topic."""
        print(f"Publishing {self.ticker} ...")
        msg = Int64()  # Create a message object of type Int64.
        msg.data = self.ticker
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
    sys.Add(floor)  # Add the floor to the simulation system.

    # Create a box object.
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 5))  # Position the box above the floor.
    box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))  # Rotate the box slightly.
    box.SetName("box")  # Set the name for ROS communication.
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
    custom_handler = MyCustomHandler("~/my_topic")
    ros_manager.RegisterPythonHandler(custom_handler)

    # Initialize the ROS manager.
    ros_manager.Initialize()

    # Set up Irrlicht visualization
    irr_options = ch.ChIrrAppearance()
    irr_options.SetTexture(ch.ChIrrAppearance.TEXTURE_FILE_PLAIN_RGB, "path/to/floor_texture.png")
    floor.SetAppearance(irr_options)
    irr_options.SetTexture(ch.ChIrrAppearance.TEXTURE_FILE_PLAIN_RGB, "path/to/box_texture.png")
    box.SetAppearance(irr_options)

    # Create and configure the Irrlicht renderer.
    irr_renderer = ch.ChIrrAppearance()
    irr_renderer.SetCamera(ch.ChIrrAppearance.CAMERA_PERSPECTIVE)
    irr_renderer.SetLighting(ch.ChIrrAppearance.LIGHTING_ALL)
    irr_renderer.SetWindowProperties(ch.ChIrrAppearance.WINDOW_WIDTH, 800, ch.ChIrrAppearance.WINDOW_HEIGHT, 600, ch.ChIrrAppearance.WINDOW_TITLE, "My Simulation")
    irr_renderer.AddCamera(ch.ChIrrAppearance.CAMERA_DEFAULT, ch.ChIrrAppearance.CAMERA_POSITION, ch.ChVector3d(0, 0, 10))
    irr_renderer.AddCamera(ch.ChIrrAppearance.CAMERA_RIGHT, ch.ChIrrAppearance.CAMERA_POSITION, ch.ChVector3d(10, 0, 0))
    irr_renderer.AddLight(ch.ChIrrAppearance.LIGHT_POSITION, ch.ChVector3d(0, 0, 10))
    irr_renderer.AddLight(ch.ChIrrAppearance.LIGHT_POSITION, ch.ChVector3d(0, 10, 0))
    irr_renderer.AddLight(ch.ChIrrAppearance.LIGHT_POSITION, ch.ChVector3d(0, -10, 0))
    irr_renderer.AddLight(ch.ChIrrAppearance.LIGHT_POSITION, ch.ChVector3d(-10, 0, 0))
    irr_renderer.AddLight(ch.ChIrrAppearance.LIGHT_POSITION, ch.ChVector3d(0, 0, -10))

    # Create and configure the Irrlicht window.
    irr_window = ch.ChIrrWindow()
    irr_window.SetWindowTitle("My Simulation")
    irr_window.SetWindowSize(irr_renderer.GetWindowWidth(), irr_renderer.GetWindowHeight())
    irr_window.SetWindowPosition(0, 0)
    irr_window.AddRenderer(irr_renderer)

    # Set up rendering frame rate control variables
    step_number = 0
    render_step_size = 1e-3  # Define the simulation time step.
    render_steps = 10  # Define the number of steps to render.

    # Run the simulation loop.
    time = 0
    time_end = 30  # Set the duration for the simulation.
    time_step = 1e-3  # Define the simulation time step.
    while time < time_end:
        sys.DoStepDynamics(time_step)  # Advance the simulation by one time step.
        time = sys.GetChTime()  # Update the simulation time.

        if not ros_manager.Update(time, time_step):  # Update ROS communication.
            break  # Exit the loop if the ROS manager indicates a problem.

        if step_number % render_steps == 0:
            irr_window.Render()  # Render the scene every few steps.
        step_number += 1

        realtime_timer = ch.ChRealtimeStepTimer()  # Create a real-time step timer.
        realtime_timer.Spin(time_step)  # Maintain real-time step execution.

# Entry point of the script.
if __name__ == "__main__":
    main()