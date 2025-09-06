import pychrono as ch
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros
import rclpy.publisher
from std_msgs.msg import Int64

# Define a custom ROS handler for publishing integer messages.
class MyCustomHandler(chros.ChROSHandler):
    """This custom handler will publish integer messages to a specified ROS topic."""

    def __init__(self, topic, rate=10):
        super().__init__(rate)  # Initialize the handler with the specified rate.
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
        msg = Int64()
        msg.data = self.ticker
        self.publisher.publish(msg)
        self.ticker += 1

def main():
    # Create the Chrono simulation system.
    sys = ch.ChSystemNSC()
    sys.SetGravitationalAcceleration(ch.ChVectorD(0, 0, -9.81))  # Correct vector type.

    # Define physical material properties for contact.
    phys_mat = ch.ChContactMaterialNSC()
    phys_mat.SetFriction(0.5)

    # Create a floor object.
    floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
    floor.SetPos(ch.ChVectorD(0, 0, -1))  # Correct vector type.
    floor.SetFixed(True)
    floor.SetName("base_link")
    sys.Add(floor)

    # Create a box object.
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVectorD(0, 0, 5))
    box.SetRot(ch.Q_from_AngAxis(0.2, ch.ChVectorD(1, 0, 0)))  # Correct quaternion creation.
    box.SetName("box")
    sys.Add(box)

    # Visualization setup
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1280, 1024)
    vis.SetWindowTitle('ROS-PyChrono Simulation')
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chronoirr.ChVectorD(0, 3, 10))  # Position camera
    vis.AddTypicalLights()
    vis.Initialize()

    # Set textures
    floor_texture = chrono.GetChronoDataFile('textures/concrete.jpg')
    box_texture = chrono.GetChronoDataFile('textures/wood.jpg')
    floor.GetVisualShape(0).SetTexture(floor_texture)
    box.GetVisualShape(0).SetTexture(box_texture)

    # ROS configuration
    publish_rate = 10  # Hz

    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler(publish_rate))
    
    # Body handler with publish rate
    ros_body_handler = chros.ChROSBodyHandler(publish_rate, box, "~/box")
    ros_manager.RegisterHandler(ros_body_handler)
    
    # Transform handler with publish rate
    tf_handler = chros.ChROSTFHandler(publish_rate)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)
    
    # Custom handler with publish rate
    custom_handler = MyCustomHandler("~/my_topic", publish_rate)
    ros_manager.RegisterPythonHandler(custom_handler)

    ros_manager.Initialize()

    # Simulation control variables
    time_step = 0.001
    time_end = 30.0
    step_number = 0
    render_step_size = 0.033  # Target 30 FPS
    render_steps = int(render_step_size / time_step)
    realtime_timer = ch.ChRealtimeStepTimer()

    time = 0.0
    while time < time_end and vis.Run():
        sys.DoStepDynamics(time_step)
        time = sys.GetChTime()

        if not ros_manager.Update(time, time_step):
            break

        # Conditional rendering
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
        step_number += 1

        realtime_timer.Spin(time_step)

if __name__ == "__main__":
    main()