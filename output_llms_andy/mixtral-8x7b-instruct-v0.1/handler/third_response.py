import pychrono as ch
import pychrono.ros as chros
import pyrr
import Irrlicht

Irr.IrrlichtDevice.setWindowCaption(u"PyChrono Simulation")
Irr.IrrlichtDevice.setResizable(True)
Irr.IrrlichtDevice.setEventReceiver(sys)

driver = Irr.createDevice(Irr.EDT_OPENGL, Irr.size(800, 600))
smgr = driver.getSceneManager()

# Create a camera.
camera = smgr.addCameraSceneNodeFPS()
camera.setPosition(ch.ChVectorD(10, 10, 10))
camera.setTarget(ch.ChVectorD(0, 0, 0))

# Create lights.
ambient_light = smgr.addAmbientLight()
ambient_light.setColor(Irr.SColor(255, 100, 100, 100))

point_light = smgr.addLightSceneNode(0, ch.ChVectorD(0, 10, 0))
point_light.setPosition(ch.ChVectorD(0, 10, 0))
point_light.setDiffuseColor(Irr.SColor(255, 255, 255, 255))

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

publish_rate = 10  # Set the ROS handlers' publishing rate to 10 Hz.
step_number = 0  # Initialize a step counter.
render_step_size = 5  # Set the number of steps between each rendering update.
render_steps = 10  # Set the number of steps to run before updating the scene.

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
    floor.SetTexture(driver, u"floor.jpg")  # Set texture for the floor.
    sys.Add(floor)  # Add the floor to the simulation system.

    # Create a box object.
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 5))  # Position the box above the floor.
    box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))  # Rotate the box slightly.
    box.SetName("box")  # Set the name for ROS communication.
    box.SetTexture(driver, u"box.jpg")  # Set texture for the box.
    sys.Add(box)  # Add the box to the simulation system.

    # Create and configure the ROS manager.
    ros_manager = chros.ChROSPythonManager()

    # Register a clock handler for the simulation time.
    ros_manager.RegisterHandler(chros.ChROSClockHandler())

    # Register a body handler to communicate the box's state.
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(publish_rate, box, "~/box"))

    # Create and register a transform handler for coordinate transformations.
    tf_handler = chros.ChROSTFHandler(publish_rate)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)

    # Register the custom handler to publish messages.
    custom_handler = MyCustomHandler("~/my_topic")
    ros_manager.RegisterPythonHandler(custom_handler)

    # Initialize the ROS manager.
    ros_manager.Initialize()

    # Run the simulation loop.
    time = 0
    time_step = 1e-3  # Define the simulation time step.
    time_end = 30  # Set the duration for the simulation.

    realtime_timer = ch.ChRealtimeStepTimer()  # Create a real-time step timer.
    while time < time_end:
        sys.DoStepDynamics(time_step)  # Advance the simulation by one time step.
        time = sys.GetChTime()  # Update the simulation time.

        step_number += 1  # Increment the step counter.

        if step_number % render_steps == 0:  # Update the scene every few steps.
            # Clear the scene.
            smgr.clear()

            # Add the floor and box to the scene.
            floor_node = ch.ChIrrMeshVisualization.AddVisualization(sys, floor, smgr)
            box_node = ch.ChIrrMeshVisualization.AddVisualization(sys, box, smgr)

            # Set the camera position and target.
            camera_position = ch.ChVectorD(floor.GetPos().x, 5, floor.GetPos().z + 5)
            camera.setPosition(camera_position)
            camera.setTarget(floor.GetPos())

            # Render the scene.
            driver.beginScene(True, True, Irr.SColor(255, 128, 128, 128))
            smgr.drawAll()
            driver.endScene()

        if not ros_manager.Update(time, time_step):  # Update ROS communication.
            break  # Exit the loop if the ROS manager indicates a problem.

        realtime_timer.Spin(time_step)  # Maintain real-time step execution.

# Entry point of the script.
if __name__ == "__main__":
    main()