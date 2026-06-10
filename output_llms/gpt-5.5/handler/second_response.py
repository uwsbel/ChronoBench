import pychrono as ch
import pychrono.ros as chros

import rclpy.publisher
from std_msgs.msg import String


# Define a custom ROS handler for publishing string messages.
class MyCustomHandler(chros.ChROSHandler):
    """This custom handler will publish string messages to a specified ROS topic."""

    def __init__(self, topic):
        super().__init__(1)  # Initialize the handler with a 1 Hz publishing rate.

        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.ticker = 0  # Initialize a counter for published messages.
        self.message = "Hello, world! At time: "

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        """Initialize the ROS publisher."""
        print(f"Creating publisher for topic {self.topic} ...")

        # Create a ROS publisher for the specified topic.
        self.publisher = interface.GetNode().create_publisher(String, self.topic, 1)

        return True  # Return True to indicate successful initialization.

    def Tick(self, time: float):
        """Publish a string message to the ROS topic."""
        msg = String()
        msg.data = self.message + str(self.ticker)

        print(f"Publishing: {msg.data}")
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
    box.SetRot(ch.QuatFromAngleAxis(0.2, ch.ChVector3d(1, 0, 0)))
    box.SetName("box")
    sys.Add(box)

    # Create and configure the ROS manager.
    ros_manager = chros.ChROSPythonManager()

    # Register a clock handler for the simulation time.
    ros_manager.RegisterHandler(chros.ChROSClockHandler())

    # Register a body handler to communicate the box's state.
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/box"))

    # Create and register a transform handler for coordinate transformations.
    tf_handler = chros.ChROSTFHandler(30)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)

    # Register the custom handler to publish string messages.
    custom_handler = MyCustomHandler("~/my_topic")
    ros_manager.RegisterPythonHandler(custom_handler)

    # Initialize the ROS manager.
    ros_manager.Initialize()

    # Run the simulation loop.
    time_step = 1e-3
    time_end = 30

    realtime_timer = ch.ChRealtimeStepTimer()

    while sys.GetChTime() < time_end:
        sys.DoStepDynamics(time_step)
        time = sys.GetChTime()

        if not ros_manager.Update(time, time_step):
            break

        realtime_timer.Spin(time_step)


# Entry point of the script.
if __name__ == "__main__":
    main()