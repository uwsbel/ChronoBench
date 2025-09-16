import pychrono as ch
import pychrono.ros as chros
import pyrr
import Irrlicht

import rclpy.publisher
from std_msgs.msg import Int64

# Define a custom ROS handler for publishing integer messages.
class MyCustomHandler(chros.ChROSHandler):
    """This custom handler will publish integer messages to a specified ROS topic."""

    def __init__(self, topic):
        super().__init__(10)  # Initialize the handler with a 10 Hz publishing rate.

        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.ticker = 0  # Initialize a counter for published messages.

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        """Initialize the ROS publisher."""
        print(f"Creating publisher for topic {self.topic} ...")
        # Create a ROS publisher for the specified topic.
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 10)
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
    floor_geometry = ch.ChBox(10, 10, 1)
    floor_material = ch.ChMaterialSurfaceNSC(phys_mat)
    floor_body = ch.ChBody(floor_geometry, 1000, floor_material)
    floor_body.SetPos(ch.ChVector3d(0, 0, -1))  # Position the floor.
    floor_body.SetFixed(True)  # Fix the floor in place.
    floor_body.SetName("base_link")  # Set the name for ROS communication.
    floor_body.SetTexture(ch.ChTexture("path/to/floor_texture.png"))
    sys.Add(floor_body)  # Add the floor to the simulation system.

    # Create a box object.
    box_geometry = ch.ChBox(1, 1, 1)
    box_material = ch.ChMaterialSurfaceNSC(phys_mat)
    box_body = ch.ChBody(box_geometry, 1000, box_material)
    box_body.SetPos(ch.ChVector3d(0, 0, 5))  # Position the box above the floor.
    box_body.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))  # Rotate the box slightly.
    box_body.SetName("box")  # Set the name for ROS communication.
    box_body.SetTexture(ch.ChTexture("path/to/box_texture.png"))
    sys.Add(box_body)  # Add the box to the simulation system.

    # Create and configure the ROS manager.
    publish_rate = 10  # Set the ROS publish rate to 10 Hz.
    ros_manager = chros.ChROSPythonManager()

    # Register a clock handler for the simulation time.
    ros_manager.RegisterHandler(chros.ChROSClockHandler(publish_rate))

    # Register a body handler to communicate the box's state.
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(publish_rate, box_body, "~/box"))

    # Create and register a transform handler for coordinate transformations.
    tf_handler = chros.ChROSTFHandler(publish_rate)
    tf_handler.AddTransform(floor_body, floor_body.GetName(), box_body, box_body.GetName())
    ros_manager.RegisterHandler(tf_handler)

    # Register the custom handler to publish messages.
    custom_handler = MyCustomHandler("~/my_topic")
    ros_manager.RegisterPythonHandler(custom_handler)

    # Initialize the Irrlicht visualization.
    device = Irrlicht.createDevice(Irrlicht.dimension2d(800, 600), 16, False, False, False, False, False)
    device.setWindowCaption("PyChrono Simulation")
    driver = device.getVideoDriver()
    scene_manager = device.getSceneManager()
    scene_manager.setAmbientLight(Irrlicht.SColor(100, 100, 100, 100))

    # Set up the camera.
    camera_node = scene_manager.addCameraSceneNode()
    camera_node.setPosition(ch.ChVectorD(5, 5, 5))
    camera_node.setTarget(ch.ChVectorD(0, 0, 0))

    # Add lights to the scene.
    light_node1 = scene_manager.addLightSceneNode(
        0,
        ch.ChVector3d(0, 10, 0),
        Irrlicht.SColor(255, 255, 255, 255),
        1000.0,
    )
    light_node2 = scene_manager.addLightSceneNode(
        0,
        ch.ChVector3d(0, -10, 0),
        Irrlicht.SColor(255, 255, 255, 255),
        1000.0,
    )

    # Initialize the ROS manager.
    ros_manager.Initialize()

    # Define variables for controlling rendering frame rate.
    step_number = 0
    render_step_size = 5
    render_steps = 1

    # Run the simulation loop.
    time = 0
    time_step = 1e-3  # Define the simulation time step.
    time_end = 30  # Set the duration for the simulation.

    realtime_timer = ch.ChRealtimeStepTimer()  # Create a real-time step timer.
    while time < time_end:
        sys.DoStepDynamics(time_step)  # Advance the simulation by one time step.
        time = sys.GetChTime()  # Update the simulation time.

        if step_number % render_step_size == 0:
            # Update ROS communication and render the scene every few steps.
            if not ros_manager.Update(time, time_step):
                break  # Exit the loop if the ROS manager indicates a problem.

            # Clear the screen and draw the scene.
            driver.beginScene(True, True, Irrlicht.SColor(100, 100, 100, 255))
            scene_manager.drawAll()
            device.postEvent(Irrlicht.EEventType.EVENT_QUIT)
            driver.endScene()

        step_number += 1  # Increment the step number.
        realtime_timer.Spin(time_step)  # Maintain real-time step execution.

# Entry point of the script.
if __name__ == "__main__":
    main()