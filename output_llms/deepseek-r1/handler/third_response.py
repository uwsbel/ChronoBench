import pychrono as ch
import pychrono.irrlicht as ch_irr
import pychrono.ros as chros

import rclpy.publisher
from std_msgs.msg import Int64

# Define a custom ROS handler for publishing integer messages.
class MyCustomHandler(chros.ChROSHandler):
    """This custom handler will publish integer messages to a specified ROS topic."""

    def __init__(self, topic):
        super().__init__(10)  # Initialize handler with 10 Hz publishing rate
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
    vis.SetWindowTitle('Controlled Rendering Demo')
    vis.Initialize()
    vis.AddCamera(ch.ChVector3d(3, 3, 3), ch.ChVector3d(0, 0, 0))
    vis.AddTypicalLights()

    # Define physical material properties for contact
    phys_mat = ch.ChContactMaterialNSC()
    phys_mat.SetFriction(0.5)

    # Create and configure floor
    floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
    floor.SetPos(ch.ChVector3d(0, 0, -1))
    floor.SetFixed(True)
    floor.SetName("base_link")
    floor.GetVisualShape(0).SetTexture(ch.GetChronoDataFile("textures/concrete.jpg"))
    sys.Add(floor)

    # Create and configure box
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 5))
    box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))
    box.SetName("box")
    box.GetVisualShape(0).SetTexture(ch.GetChronoDataFile("textures/cubetexture.png"))
    sys.Add(box)

    # Create and configure ROS manager
    ros_manager = chros.ChROSPythonManager()
    
    # Register handlers with 10 Hz update rate
    ros_manager.RegisterHandler(chros.ChROSClockHandler(10))
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(10, box, "~/box"))
    
    # Configure TF handler with corrected parameters
    tf_handler = chros.ChROSTFHandler(10)
    tf_handler.AddTransform(floor, box, floor.GetName(), box.GetName())
    ros_manager.RegisterHandler(tf_handler)
    
    # Register custom handler
    custom_handler = MyCustomHandler("~/my_topic")
    ros_manager.RegisterPythonHandler(custom_handler)

    ros_manager.Initialize()

    # Simulation parameters
    time_step = 1e-3
    time_end = 30
    render_step_size = 1/60  # 60 Hz rendering
    render_steps = int(render_step_size / time_step)
    step_number = 0

    realtime_timer = ch.ChRealtimeStepTimer()
    while (time := sys.GetChTime()) < time_end:
        # Simulation step
        sys.DoStepDynamics(time_step)

        # ROS update
        if not ros_manager.Update(time, time_step):
            break

        # Conditional rendering
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
        vis.Update()

        # Maintain real-time synchronization
        realtime_timer.Spin(time_step)
        step_number += 1

if __name__ == "__main__":
    main()