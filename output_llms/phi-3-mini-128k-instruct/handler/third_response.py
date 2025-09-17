import pychrono as ch
import pychrono.ros as chros
import rclpy.publisher
from std_msgs.msg import Int64
from rclpy.node import Node
from rclpy.node import Publisher
from rclpy.node import Subscription
from rclpy.node import Node
from rclpy.node import get_clock
from rclpy.timer import Timer
from rclpy.time import Time
from irrlicht.scene.mesh import Mesh
from irrlicht.scene.render import RenderWindow, RenderScene, RenderDevice, RenderType
from irrlicht.scene.material import Material
from irrlicht.scene.light import DirectionalLight
from irrlicht.scene.video import VideoDriver
from irrlicht.scene.video import VideoDriverType

# Define a custom ROS handler for publishing integer messages.
class MyCustomHandler(chros.ChROSHandler):
    """This custom handler will publish integer messages to a specified ROS topic."""

    def __init__(self, topic: str, publish_rate: float):
        super().__init__(publish_rate)  # Initialize the handler with the specified publishing rate.

        self.topic = topic
        self.publisher: Publisher = None
        self.ticker = 0  # Initialize a counter for published messages.

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        """Initialize the ROS publisher."""
        print(f"Creating publisher for topic {self.topic} ...")
        # Create a ROS publisher for the specified topic with the specified rate.
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, self.publish_rate)
        return True  # Return True to indicate successful initialization.

    def Tick(self, time: float) -> None:
        """Publish an integer message to the ROS topic."""
        print(f"Publishing {self.ticker} ...")
        msg = Int64()  # Create a message object of type Int64.
        msg.data = self.ticker  # Set the message data to the current ticker value.
        self.publisher.publish(msg)  # Publish the message to the ROS topic.
        self.ticker += 1  # Increment the ticker for the next message.

# Initialize ROS node and create a publisher.
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
    floor.SetTexture("path/to/floor_texture.jpg")  # Set the texture for the floor.
    floor.SetName("base_link")  # Set the name for ROS communication.
    sys.Add(floor)  # Add the floor to the simulation system.

    # Create a box object.
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 5))  # Position the box above the floor.
    box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))  # Rotate the box slightly.
    box.SetTexture("path/to/box_texture.jpg")  # Set the texture for the box.
    box.SetName("box")  # Set the name for ROS communication.
    sys.Add(box)  # Add the box to the simulation system.

    # Initialize the ROS manager.
    ros_manager = chros.ChROSPythonManager()
    
    # Register a clock handler for the simulation time.
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    
    # Register a body handler to communicate the box's state.
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/box"))
    
    # Create and configure the Irrlicht visualization system.
    irrlicht_scene = Mesh.addFilledMesh(Mesh.createPlane(8, 8, 1))
    irrlicht_scene.SetMaterial(Material.New(0, Material.CreateMipMap(Material.New(0, 0.5, 0.5, 1, 1, 1)))
    irrlicht_scene.SetTexture("path/to/floor_texture.jpg")
    irrlicht_scene.SetTexture("path/to/box_texture.jpg")

    # Set up Irrlicht rendering device and window.
    render_device = RenderDevice(VideoDriver(VideoDriverType.DIRECTVIDEO))
    render_window = RenderWindow(800, 600, "PyChrono Simulation", "DirectWindow", render_device)
    render_scene = RenderScene(render_window, RenderType.GREENSCREEN)
    render_scene.AddLight(DirectionalLight(LightType.DIRECTIONAL, DirectionalLight.CreateBlue(255, 255, 255, 100))
    render_scene.AddCamera("irrlicht_camera", 50, 50, 50)

    # Create a timer for updating the scene every few steps.
    step_number = 0
    render_step_size = 100
    render_steps = 10
    timer = Timer(get_clock(), render_step_size, True)

    # Initialize ROS manager.
    ros_manager.Initialize()

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

        # Update the Irrlicht scene every few steps.
        if step_number % render_steps == 0:
            render_scene.RenderScene()
            timer.reset()

        # Update the ROS manager.
        ros_manager.Update(time, time_step)

        # Update the Irrlicht camera and lights.
        irrlicht_scene.SetCamera("irrlicht_camera")
        irrlicht_scene.SetLightPosition(ch.ChVector3d(10, 10, 10))

        step_number += 1

    # Clean up.
    timer.cancel()
    render_scene.DeleteAllActors()
    render_window.Close()

if __name__ == "__main__":
    rclpy.init(args=None)
    node = Node("pychrono_ros_visualization")
    custom_handler = MyCustomHandler("~/my_topic", 10)  # Set publish_rate to 10 Hz.
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterPythonHandler(custom_handler)
    ros_manager.Initialize()
    main()