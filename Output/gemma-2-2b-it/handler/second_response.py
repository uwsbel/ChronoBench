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
        self.ticker = 0  
        self.message = "Hello, world! At time: "

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        """Initialize the ROS publisher."""
        print(f"Creating publisher for topic {self.topic} ...")
        # Create a ROS publisher for the specified topic.
        self.publisher = interface.GetNode().create_publisher(String, self.topic, 1)
        return True  # Return True to indicate successful initialization.

    def Tick(self, time: float):
        """Publish a string message to the ROS topic."""
        print(f"Publishing {self.message} ...")
        msg = String()  
        msg.data = self.message + str(self.ticker)
        self.publisher.publish(msg)  
        self.ticker += 1