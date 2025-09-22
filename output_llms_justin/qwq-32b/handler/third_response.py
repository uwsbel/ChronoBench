import pychrono as ch
import pychrono.ros as chros
import pychrono.irrlicht as irr
from std_msgs.msg import Int64

# Define a custom ROS handler for publishing integer messages.
class MyCustomHandler(chros.ChROSHandler):
    """This custom handler will publish integer messages to a specified ROS topic."""
    def __init__(self, topic, rate=10):
        super().__init__(rate)  # Initialize with the specified rate.
        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.ticker = 0  # Initialize a counter for published messages.

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        """Initialize the ROS publisher."""
        print(f"Creating publisher for topic {self.topic} ...")
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
    # Setup simulation system
    sys = ch.ChSystemNSC()
    sys.SetGravitationalAcceleration(ch.ChVectorD(0, 0, -9.81))  # Set gravitational acceleration.

    # Physical material properties
    phys_mat = ch.ChContactMaterialNSC()
    phys_mat.SetFriction(0.5)

    # Create floor
    floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
    floor.SetPos(ch.ChVectorD(0, 0, -1))
    floor.SetFixed(True)
    floor.SetName("base_link")
    sys.Add(floor)

    # Create box
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVectorD(0, 0, 5))
    box.SetRot(ch.Q_from_AngAxis(0.2, ch.ChVectorD(1, 0, 0)))
    box.SetName("box")
    sys.Add(box)

    # Visualization setup
    my_vis = irr.ChIrrApp(sys, 'PyChrono Simulation', True)
    my_vis.SetCameraPosition(ch.ChVectorD(0, 3, 5))
    my_vis.SetCameraRotation(ch.ChVectorD(0, 45, 0))
    my_vis.AddTypicalLights()
    my_vis.AddLightWithDirection(ch.ChVectorD(0, 0, 1), ch.ChColor(1, 1, 1), 2)
    my_vis.AssetBind()
    my_vis.AssetUpdate()
    my_vis.SetWindowSize(1280, 1024)
    my_vis.SetWindowTitle('PyChrono ROS Simulation')

    # Add textures
    floor_texture = ch.ChTexture()
    floor_texture.SetTextureFilename("textures/floor.jpg")
    floor.AddAsset(floor_texture)

    box_texture = ch.ChTexture()
    box_texture.SetTextureFilename("textures/box.jpg")
    box.AddAsset(box_texture)

    # ROS configuration
    publish_rate = 10  # Hz
    ros_manager = chros.ChROSPythonManager()

    # Register ROS handlers with publish_rate
    ros_manager.RegisterHandler(chros.ChROSClockHandler(publish_rate))
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(publish_rate, box, "~/box"))

    tf_handler = chros.ChROSTFHandler(publish_rate)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)

    # Custom handler with publish_rate
    custom_handler = MyCustomHandler("~/my_topic", publish_rate)
    ros_manager.RegisterPythonHandler(custom_handler)

    ros_manager.Initialize()

    # Simulation parameters
    time_step = 1e-3
    time_end = 30
    render_step_size = 0.01  # seconds between renders
    render_steps = int(render_step_size / time_step)
    step_number = 0

    realtime_timer = ch.ChRealtimeStepTimer()

    while sys.GetChTime() < time_end:
        sys.DoStepDynamics(time_step)
        current_time = sys.GetChTime()

        # Update ROS
        if not ros_manager.Update(current_time, time_step):
            break

        # Rendering control
        step_number += 1
        if step_number % render_steps == 0:
            my_vis.BeginScene()
            my_vis.DrawAll()
            my_vis.EndScene()
            step_number = 0  # Reset counter

        realtime_timer.Spin(time_step)

if __name__ == "__main__":
    main()