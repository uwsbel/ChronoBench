import pychrono as ch
import pychrono.ros as chros
from std_msgs.msg import Int64

# Define a custom ROS handler for publishing integer messages.
class MyCustomHandler(chros.ChROSHandler):
    """This custom handler will publish integer messages to a specified ROS topic."""

    def __init__(self, topic, rate=10):
        super().__init__(rate)  # Initialize the handler with the specified rate.
        self.topic = topic
        self.publisher = None
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
    sys.SetGravitationalAcceleration(ch.ChVectorD(0, 0, -9.81))  # Set gravitational acceleration.

    # Define physical material properties for contact.
    phys_mat = ch.ChMaterialSurfaceNSC()
    phys_mat.SetFriction(0.5)  # Set friction coefficient.

    # Create a floor object.
    floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
    floor.SetPos(ch.ChVectorD(0, 0, -1))  # Position the floor.
    floor.SetFixed(True)  # Fix the floor in place.
    floor.SetName("base_link")  # Set the name for ROS communication.
    sys.Add(floor)  # Add the floor to the simulation system.

    # Create a box object.
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVectorD(0, 0, 5))  # Position the box above the floor.
    box.SetRot(ch.Q_from_AngAxis(0.2, ch.ChVectorD(1, 0, 0)))  # Rotate the box slightly.
    box.SetName("box")  # Set the name for ROS communication.
    sys.Add(box)  # Add the box to the simulation system.

    # Set textures for floor and box
    # Assuming texture files exist in the 'textures' directory
    floor_shape = floor.GetVisualShape(0)
    floor_texture = ch.ChTexture()
    floor_texture.SetPath("textures/floor.jpg")
    floor_shape.SetTexture(floor_texture)

    box_shape = box.GetVisualShape(0)
    box_texture = ch.ChTexture()
    box_texture.SetPath("textures/box.jpg")
    box_shape.SetTexture(box_texture)

    # Visualization setup with Irrlicht
    my_vis = ch.ChIrrApp(sys, 'PyChrono ROS Demo', True)
    my_vis.SetWindowSize(1280, 1024)
    my_vis.SetWindowTitle('My Simulation')
    my_vis.SetCamPosition(ch.ChVectorD(0, 5, 10))
    my_vis.SetCamRotation(ch.Q_from_AngAxis(0.2, ch.ChVectorD(1, 0, 0)))
    my_vis.AddTypicalLights()
    my_vis.AllocateResources()
    my_vis.AssetBind()
    my_vis.AssetUpdate()
    my_vis.SetTimestep(1e-3)  # Set the simulation time step for visualization

    # ROS configuration variables
    publish_rate = 10  # Hz

    # Create and configure the ROS manager.
    ros_manager = chros.ChROSPythonManager()

    # Register a clock handler for the simulation time.
    clock_handler = chros.ChROSClockHandler(publish_rate)
    ros_manager.RegisterHandler(clock_handler)

    # Register a body handler to communicate the box's state.
    body_handler = chros.ChROSBodyHandler(publish_rate, box, "~/box")
    ros_manager.RegisterHandler(body_handler)

    # Create and register a transform handler for coordinate transformations.
    tf_handler = chros.ChROSTFHandler(publish_rate)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)

    # Register the custom handler to publish messages.
    custom_handler = MyCustomHandler("~/my_topic", publish_rate)
    ros_manager.RegisterPythonHandler(custom_handler)

    # Initialize the ROS manager.
    ros_manager.Initialize()

    # Simulation control variables
    time_step = 1e-3  # Simulation time step
    time_end = 30     # Simulation duration
    render_steps = 10  # Number of simulation steps between renders
    step_number = 0

    # Real-time step timer
    realtime_timer = ch.ChRealtimeStepTimer()

    # Simulation loop
    time = 0
    while time < time_end:
        sys.DoStepDynamics(time_step)
        time = sys.GetChTime()

        # Update ROS communication
        if not ros_manager.Update(time, time_step):
            break

        # Conditional rendering
        step_number += 1
        if step_number % render_steps == 0:
            my_vis.BeginScene()
            my_vis.DrawAll()
            my_vis.EndScene()

        # Maintain real-time step execution
        realtime_timer.Spin(time_step)

    # End simulation
    my_vis.Close()

if __name__ == "__main__":
    main()

import pychrono as ch
import pychrono.ros as chros
from std_msgs.msg import Int64

class MyCustomHandler(chros.ChROSHandler):
    """Custom handler for publishing integer messages."""
    def __init__(self, topic, rate=10):
        super().__init__(rate)
        self.topic = topic
        self.publisher = None
        self.ticker = 0

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        print(f"Creating publisher for {self.topic}...")
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 1)
        return True

    def Tick(self, time: float):
        print(f"Publishing {self.ticker}...")
        msg = Int64()
        msg.data = self.ticker
        self.publisher.publish(msg)
        self.ticker += 1

def main():
    sys = ch.ChSystemNSC()
    sys.SetGravitationalAcceleration(ch.ChVectorD(0, 0, -9.81))

    # Materials and bodies
    phys_mat = ch.ChMaterialSurfaceNSC()
    phys_mat.SetFriction(0.5)

    floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
    floor.SetPos(ch.ChVectorD(0, 0, -1))
    floor.SetFixed(True)
    floor.SetName("base_link")
    sys.Add(floor)

    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVectorD(0, 0, 5))
    box.SetRot(ch.Q_from_AngAxis(0.2, ch.ChVectorD(1, 0, 0)))
    box.SetName("box")
    sys.Add(box)

    # Textures
    floor_shape = floor.GetVisualShape(0)
    floor_texture = ch.ChTexture()
    floor_texture.SetPath("textures/floor.jpg")
    floor_shape.SetTexture(floor_texture)

    box_shape = box.GetVisualShape(0)
    box_texture = ch.ChTexture()
    box_texture.SetPath("textures/box.jpg")
    box_shape.SetTexture(box_texture)

    # Irrlicht setup
    my_vis = ch.ChIrrApp(sys, 'PyChrono ROS Demo', True)
    my_vis.SetWindowSize(1280, 1024)
    my_vis.SetWindowTitle('My Simulation')
    my_vis.SetCamPosition(ch.ChVectorD(0, 5, 10))
    my_vis.SetCamRotation(ch.Q_from_AngAxis(0.2, ch.ChVectorD(1, 0, 0)))
    my_vis.AddTypicalLights()
    my_vis.AllocateResources()
    my_vis.AssetBind()
    my_vis.AssetUpdate()
    my_vis.SetTimestep(1e-3)

    # ROS configuration
    publish_rate = 10
    ros_manager = chros.ChROSPythonManager()

    # ROS handlers
    clock_handler = chros.ChROSClockHandler(publish_rate)
    ros_manager.RegisterHandler(clock_handler)

    body_handler = chros.ChROSBodyHandler(publish_rate, box, "~/box")
    ros_manager.RegisterHandler(body_handler)

    tf_handler = chros.ChROSTFHandler(publish_rate)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)

    custom_handler = MyCustomHandler("~/my_topic", publish_rate)
    ros_manager.RegisterPythonHandler(custom_handler)

    ros_manager.Initialize()

    # Simulation parameters
    time_step = 1e-3
    time_end = 30
    render_steps = 10
    step_number = 0
    realtime_timer = ch.ChRealtimeStepTimer()

    time = 0
    while time < time_end:
        sys.DoStepDynamics(time_step)
        time = sys.GetChTime()
        if not ros_manager.Update(time, time_step):
            break

        step_number += 1
        if step_number % render_steps == 0:
            my_vis.BeginScene()
            my_vis.DrawAll()
            my_vis.EndScene()

        realtime_timer.Spin(time_step)

    my_vis.Close()

if __name__ == "__main__":
    main()