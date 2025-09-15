import pychrono as ch
import pychrono.ros as chros
import pychrono irrlicht as ch irr  # Fix: Proper import for Irrlicht

import rclpy.publisher
from std_msgs.msg import Int64

# Define a custom ROS handler for publishing integer messages.
class MyCustomHandler(chros.ChROSHandler):
    """This custom handler will publish integer messages to a specified ROS topic."""

    def __init__(self, topic):
        super().__init__(10)  # Fix: Set publish rate to 10 Hz as per instructions
        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.ticker = 0  # Initialize a counter for published messages.

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
    # Create the Chrono simulation system.
    sys = ch.ChSystemNSC()
    sys.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.81))

    # Define physical material properties for contact.
    phys_mat = ch.ChContactMaterialNSC()
    phys_mat.SetFriction(0.5)

    # Create a floor object.
    floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
    floor.SetPos(ch.ChVector3d(0, 0, -1))
    floor.SetFixed(True)
    floor.SetName("base_link")
    sys.Add(floor)

    # Create a box object.
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 5))
    box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))
    box.SetName("box")
    sys.Add(box)

    # Visualization setup with Irrlicht
    vis = ch irr.ChIrrApp(sys, "Simulation Window", True)
    vis.SetWindowSize(800, 600)
    vis.SetWindowTitle("Simulation Window")
    vis.AddLightWithShadow(ch.ChVector3d(0, 1, 3), ch.ChVector3d(0, 0, 0), 10, 100, 500)
    vis.AddTypicalLogo()
    vis.AddTypicalSky()
    vis.AddTypicalGrid()
    vis.AddTypicalAxes()
    vis.SetCameraPosition(ch.ChVector3d(0, 5, 5))
    vis.SetCameraRotation(ch.ChQuaterniond(0, 1, 0, 0))

    # Set textures for floor and box
    floor.SetTexture(ch irr.ChTexture())
    floor.GetTexture().SetPath("textures/concrete.jpg")
    box.SetTexture(ch irr.ChTexture())
    box.GetTexture().SetPath("textures/wood.jpg")

    # Create and configure the ROS manager.
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    
    # Set publish rate to 10 Hz for ROS handlers
    publish_rate = 10
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(publish_rate, box, "~/box"))
    
    # Create and register a transform handler
    tf_handler = chros.ChROSTFHandler(publish_rate)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)
    
    # Register the custom handler
    custom_handler = MyCustomHandler("~/my_topic")
    ros_manager.RegisterPythonHandler(custom_handler)

    # Initialize the ROS manager.
    ros_manager.Initialize()

    # Initialize visualization
    vis.Initialize()

    # Variables for controlling rendering frame rate
    step_number = 0
    render_step_size = 10  # Render every 10 steps
    render_steps = 0

    # Run the simulation loop.
    time = 0
    time_step = 1e-3
    time_end = 30
    realtime_timer = ch.ChRealtimeStepTimer()

    while time < time_end:
        sys.DoStepDynamics(time_step)
        time = sys.GetChTime()

        if not ros_manager.Update(time, time_step):
            break

        realtime_timer.Spin(time_step)

        # Update visualization
        step_number += 1
        if step_number % render_step_size == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            render_steps += 1

    # Close visualization
    vis.Close()

# Entry point of the script.
if __name__ == "__main__":
    main()