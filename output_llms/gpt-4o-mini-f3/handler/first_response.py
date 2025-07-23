import pychrono as chrono
import pychrono.ros as crosevo
import rclpy.publisher
from std_msgs.msg import Int64

# Define a custom ROS handler for publishing integer messages.
class MyCustomHandler(crosevo.ChROSHandler):
    """This custom handler will publish integer messages to a specified ROS topic."""

    def __init__(self, topic):
        super().__init__(1)  # Initialize the handler with a 1 Hz publishing rate.

        self.topic = topic
        self.topic_daytime = topic + "/daytime"
        self.publisher: rclpy.publisher.Publisher = None
        self.int64_msg = Int64()

    def Initialize(self, interface: crosevo.ChROSPythonInterface) -> bool:
        """Initialize the ROS publisher."""
        print("Creating publisher for topic:", self.topic)
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 1)
        return True  # Return True to indicate successful initialization.

    def Update(self, time: float):
        """Publish the current time as an integer message."""
        self.int64_msg.data = int(time)
        print("Publishing:", self.int64_msg.data, "from topic:", self.topic)
        self.publisher.publish(self.int64_msg)  # Publish the integer message.

def main():
    # Create the Chrono simulation system.
    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
    chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

    # Add a floor to the simulation.
    floor_mat = chrono.ChContactMaterialNSC()
    floor = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, floor_mat)
    floor.SetPos(chrono.ChVector3d(0, 0, -1))  # Position the floor.
    floor.SetFixed(True)  # Fix the floor in place.
    floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    sys.Add(floor)

    # Add a box to the simulation.
    box_mat = chrono.ChContactMaterialNSC()
    box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, box_mat)
    box.SetPos(chrono.ChVector3d(0, 0, 5))  # Position the box above the floor.
    box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    sys.Add(box)

    # Create and configure the ROS manager.
    ros_manager = crosevo.ChROSPythonManager()
    ros_manager.RegisterHandler(MyCustomHandler("/pychrono/box/topic"))

    # Add a clock handler for the simulation time.
    ros_manager.RegisterHandler(crosevo.ChROSClockHandler())

    # Add a body handler to publish the box's state.
    body_handler = crosevo.ChROSBodyHandler(ros_manager, box, "/pychrono/box")
    body_handler.RegisterFantasyLabel("/fancy/label/here")

    # Add a transform handler for publishing transforms.
    tf_handler = crosevo.ChROSTFHandler(ros_manager)
    tf_handler.RegisterTransformBetweenFrames(box, "", box.GetName(), "")

    # Initialize the ROS manager.
    ros_manager.Initialize()

    # Run the simulation loop.
    time = 0
    time_step = 1e-3
    time_end = 30

    realtime_timer = chrono.ChRealtimeStepTimer()  # Create a real-time step timer.
    while time < time_end:
        sys.DoStepDynamics(time_step)  # Advance the simulation by one time step.
        time = sys.GetChTime()  # Update the simulation time.

        if not ros_manager.Update(time, time_step):  # Update ROS communication.
            break  # Exit the loop if the ROS manager indicates a problem.

        realtime_timer.Spin(time_step)  # Maintain real-time step execution.

# Entry point of the script.
if __name__ == "__main__":
    main()