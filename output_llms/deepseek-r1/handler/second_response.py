import pychrono as ch
import pychrono.ros as chros

import rclpy.publisher
from std_msgs.msg import String  # Changed from Int64 to String

# Define a custom ROS handler for publishing string messages.
class MyCustomHandler(chros.ChROSHandler):
    """This custom handler will publish string messages to a specified ROS topic."""

    def __init__(self, topic):
        super().__init__(1)  # Initialize the handler with a 1 Hz publishing rate.
        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.ticker = 0  # Initialize a counter for published messages.
        self.message = "Hello, world! At time: "  # Added message attribute

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        """Initialize the ROS publisher."""
        print(f"Creating publisher for topic {self.topic} ...")
        # Create a ROS publisher for the specified topic with String type
        self.publisher = interface.GetNode().create_publisher(String, self.topic, 1)
        return True  # Return True to indicate successful initialization.

    def Tick(self, time: float):
        """Publish a string message to the ROS topic."""
        print(f"Publishing {self.ticker} ...")
        msg = String()  # Create message object of type String
        msg.data = self.message + str(self.ticker)  # Concatenate message with ticker
        self.publisher.publish(msg)  # Publish the message to the ROS topic
        self.ticker += 1  # Increment the ticker for the next message

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

    # Create and configure the ROS manager
    ros_manager = chros.ChROSPythonManager()
    
    # Register handlers
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/box"))
    
    tf_handler = chros.ChROSTFHandler(30)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)
    
    custom_handler = MyCustomHandler("~/my_topic")
    ros_manager.RegisterPythonHandler(custom_handler)

    ros_manager.Initialize()

    # Run simulation loop
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

if __name__ == "__main__":
    main()