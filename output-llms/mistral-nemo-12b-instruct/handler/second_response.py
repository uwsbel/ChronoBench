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
        self.message = "Hello, world! At time: "  # Added message attribute
        self.ticker = 0  # Initialize a counter for published messages.

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        """Initialize the ROS publisher."""
        print(f"Creating publisher for topic {self.topic} ...")
        # Create a ROS publisher for the specified topic.
        self.publisher = interface.GetNode().create_publisher(String, self.topic, 1)
        return True  # Return True to indicate successful initialization.

    def Tick(self, time: float):
        """Publish a string message to the ROS topic."""
        print(f"Publishing {self.message + str(self.ticker)} ...")
        msg = String()  # Create a message object of type String.
        msg.data = self.message + str(self.ticker)  # Set the message data to the concatenated string.
        self.publisher.publish(msg)  # Publish the message to the ROS topic.
        self.ticker += 1  # Increment the ticker for the next message.

def main():
    # ... (rest of the main function remains unchanged)

    # Register the custom handler to publish messages.
    custom_handler = MyCustomHandler("~/my_topic")
    ros_manager.RegisterPythonHandler(custom_handler)

    # ... (rest of the main function remains unchanged)

if __name__ == "__main__":
    main()