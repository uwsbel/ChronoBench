import pychrono as ch
import pychrono.ros as chros
import pychrono.irrlicht as chrirr  # Import Irrlicht module
from rclpy.publisher import Publisher  # Corrected publisher import
from std_msgs.msg import Int64

# Define a custom ROS handler for publishing integer messages.
class MyCustomHandler(chros.ChROSHandler):
    """This custom handler will publish integer messages to a specified ROS topic."""

    def __init__(self, topic, rate):  # Added rate parameter
        super().__init__(rate)  # Use specified rate instead of fixed 1 Hz
        self.topic = topic
        self.publisher: Publisher = None
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

    # Define physical material properties for contact
    phys_mat = ch.ChContactMaterialNSC()
    phys_mat.SetFriction(0.5)

    # Create a floor object
    floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
    floor.SetPos(ch.ChVector3d(0, 0, -1))
    floor.SetFixed(True)
    floor.SetName("base_link")
    sys.Add(floor)

    # Create a box object
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 5))
    box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))
    box.SetName("box")
    sys.Add(box)

    # Set textures for visualization
    floor_texture = ch.GetChronoDataFile("textures/concrete.jpg")
    box_texture = ch.GetChronoDataFile("textures/cubetexture_bluewhite.png")
    floor.GetVisualShape(0).SetTexture(floor_texture)
    box.GetVisualShape(0).SetTexture(box_texture)

    # Create and configure Irrlicht visualization
    vis = chrirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('PyChrono Simulation')
    vis.Initialize()
    vis.AddCamera(ch.ChVector3d(0, 3, 3), ch.ChVector3d(0, 0, 0))  # Camera position and target
    vis.AddTypicalLights()
    vis.AddSkyBox()

    # Create and configure ROS manager with publish rate
    publish_rate = 10  # 10 Hz publishing rate
    ros_manager = chros.ChROSPythonManager()
    
    # Register handlers with publish_rate
    ros_manager.RegisterHandler(chros.ChROSClockHandler(publish_rate))
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(publish_rate, box, "~/box"))
    
    # Create and register transform handler
    tf_handler = chros.ChROSTFHandler(publish_rate)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)
    
    # Register custom handler with publish_rate
    custom_handler = MyCustomHandler("~/my_topic", publish_rate)
    ros_manager.RegisterPythonHandler(custom_handler)

    # Initialize ROS manager
    ros_manager.Initialize()

    # Simulation parameters
    time = 0
    time_step = 1e-3
    time_end = 30
    step_number = 0  # Track simulation steps
    render_step_size = 1.0 / 60  # 60 FPS rendering
    render_steps = int(render_step_size / time_step)  # Steps between renders

    realtime_timer = ch.ChRealtimeStepTimer()
    while time < time_end:
        # Handle visualization events
        if not vis.Run():
            break
        
        # Advance simulation
        sys.DoStepDynamics(time_step)
        time = sys.GetChTime()

        # Update ROS
        if not ros_manager.Update(time, time_step):
            break

        # Render every render_steps
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
        
        step_number += 1
        realtime_timer.Spin(time_step)

if __name__ == "__main__":
    main()