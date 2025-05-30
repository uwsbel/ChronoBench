import pychrono as ch
import pychrono.irrlicht as ch_irr
import pychrono.ros as chros

import rclpy.publisher
from std_msgs.msg import Int64

# Define a custom ROS handler for publishing integer messages.
class MyCustomHandler(chros.ChROSHandler):
    """This custom handler will publish integer messages to a specified ROS topic."""

    def __init__(self, topic, rate):
        super().__init__(rate)  # Use provided rate for publishing
        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.ticker = 0

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        """Initialize the ROS publisher."""
        print(f"Creating publisher for topic {self.topic} ...")
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 1)
        return True

    def Tick(self, time: float):
        """Publish an integer message to the ROS topic."""
        print(f"Publishing {self.ticker} ...")
        msg = Int64()
        msg.data = self.ticker
        self.publisher.publish(msg)
        self.ticker += 1

def main():
    # Create the Chrono simulation system
    sys = ch.ChSystemNSC()
    sys.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.81))

    # Set up Irrlicht visualization
    vis = ch_irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Modified Simulation')
    vis.Initialize()
    vis.AddCamera(ch.ChVector3d(0, 3, 2), ch.ChVector3d(0, 0, 0))
    vis.AddTypicalLights()

    # Define physical material properties
    phys_mat = ch.ChContactMaterialNSC()
    phys_mat.SetFriction(0.5)

    # Create floor with texture
    floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
    floor.SetPos(ch.ChVector3d(0, 0, -1))
    floor.SetFixed(True)
    floor.SetName("base_link")
    floor.GetVisualShape(0).SetTexture(ch.GetChronoDataFile("textures/concrete.jpg"))
    sys.Add(floor)

    # Create box with texture
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 5))
    box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))
    box.SetName("box")
    box.GetVisualShape(0).SetTexture(ch.GetChronoDataFile("textures/blue.png"))
    sys.Add(box)

    # ROS configuration
    publish_rate = 10  # 10 Hz publishing rate
    ros_manager = chros.ChROSPythonManager()
    
    # Register handlers with publish_rate
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(publish_rate, box, "~/box"))
    
    tf_handler = chros.ChROSTFHandler(publish_rate)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)
    
    custom_handler = MyCustomHandler("~/my_topic", publish_rate)
    ros_manager.RegisterPythonHandler(custom_handler)

    ros_manager.Initialize()

    # Simulation loop configuration
    time = 0
    time_step = 1e-3
    time_end = 30
    
    # Rendering control variables
    step_number = 0
    render_step_size = 1.0 / 60  # 60 FPS
    render_steps = int(render_step_size / time_step)

    realtime_timer = ch.ChRealtimeStepTimer()
    while time < time_end:
        # Simulation step
        sys.DoStepDynamics(time_step)
        time = sys.GetChTime()

        # ROS update
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