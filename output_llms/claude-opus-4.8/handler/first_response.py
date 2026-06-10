import pychrono as ch
import pychrono.ros as chros

from rclpy.publisher import Publisher
from std_msgs.msg import Int64


class MyCustomHandler(chros.ChROSHandler):
    """
    Custom ROS handler that publishes integer messages to a specified topic.
    Extends the base ChROSHandler class.
    """

    def __init__(self, topic):
        # Initialize the base handler with a publish rate (Hz)
        super().__init__(25)  # Publish at 25 Hz

        self.topic = topic
        self.publisher: Publisher = None
        self.ticker = 0  # Counter for the integer messages

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        """
        Initialize the publisher. Called once during ROS manager initialization.
        """
        print(f"Creating publisher for topic '{self.topic}' ...")

        # Create a ROS publisher using the interface's node
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 1)

        return True

    def Tick(self, time: float):
        """
        Called at each update step to publish a message.
        """
        print(f"Publishing message at time {time:.3f} ...")

        # Construct and populate the message
        msg = Int64()
        msg.data = self.ticker

        # Publish the message
        self.publisher.publish(msg)

        # Increment the counter
        self.ticker += 1


def main():
    # ---------------------------------------------------------------------
    # 1. Create the Chrono physical system
    # ---------------------------------------------------------------------
    system = ch.ChSystemNSC()

    # Set gravitational acceleration (standard Earth gravity along -Z)
    system.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.81))

    # ---------------------------------------------------------------------
    # 2. Define physical material properties
    # ---------------------------------------------------------------------
    material = ch.ChContactMaterialNSC()
    material.SetFriction(0.5)
    material.SetRestitution(0.1)

    # ---------------------------------------------------------------------
    # 3. Add a fixed floor to the system
    # ---------------------------------------------------------------------
    floor = ch.ChBodyEasyBox(
        10, 10, 1,      # dimensions (x, y, z)
        1000,           # density
        True,           # create visualization asset
        True,           # enable collision
        material        # contact material
    )
    floor.SetPos(ch.ChVector3d(0, 0, -1))  # position below the box
    floor.SetFixed(True)                   # make the floor immovable
    floor.SetName("base_link")             # name used by transform handler
    system.Add(floor)

    # ---------------------------------------------------------------------
    # 4. Add a movable box to the system
    # ---------------------------------------------------------------------
    box = ch.ChBodyEasyBox(
        1, 1, 1,        # dimensions (x, y, z)
        1000,           # density
        True,           # create visualization asset
        True,           # enable collision
        material        # contact material
    )
    box.SetPos(ch.ChVector3d(0, 0, 5))     # start above the floor
    box.SetName("box")                     # name used by handlers
    system.Add(box)

    # ---------------------------------------------------------------------
    # 5. Create the ROS manager and configure handlers
    # ---------------------------------------------------------------------
    ros_manager = chros.ChROSPythonManager()

    # Clock handler: publishes simulation time to /clock
    ros_manager.RegisterHandler(chros.ChROSClockHandler())

    # Body handler: publishes the box state (pose, velocity, etc.)
    ros_manager.RegisterHandler(
        chros.ChROSBodyHandler(25, box, "~/box_state")
    )

    # Transform (TF) handler: publishes coordinate transforms between bodies
    ros_manager.RegisterHandler(chros.ChROSTFHandler(25))

    # Register the custom integer-publishing handler
    custom_handler = MyCustomHandler("~/my_topic")
    ros_manager.RegisterPythonHandler(custom_handler)

    # Initialize the ROS manager (sets up node, publishers, etc.)
    ros_manager.Initialize()

    # ---------------------------------------------------------------------
    # 6. Run the simulation loop
    # ---------------------------------------------------------------------
    time = 0.0
    time_step = 1e-3   # 1 ms timestep
    time_end = 30.0    # total simulation time (seconds)

    # Real-time timer to keep simulation in sync with wall-clock time
    realtime_timer = ch.ChRealtimeStepTimer()

    while time < time_end:
        # Advance the physical system by one timestep
        system.DoStepDynamics(time_step)
        time = system.GetChTime()

        # Update ROS communication (publish messages, process callbacks)
        if not ros_manager.Update(time, time_step):
            break

        # Maintain real-time execution
        realtime_timer.Spin(time_step)


if __name__ == "__main__":
    main()