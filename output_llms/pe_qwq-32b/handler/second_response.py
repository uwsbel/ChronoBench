import pychrono as ch
import pychrono.ros as chros

from std_msgs.msg import String  # Changed message type to String

class MyCustomHandler(chros.ChROSHandler):
    """Custom handler to publish string messages."""
    def __init__(self, topic):
        super().__init__(1)  # 1 Hz publishing rate
        self.topic = topic
        self.publisher = None
        self.ticker = 0
        self.message = "Hello, world! At time: "  # Added message attribute

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        """Initialize ROS publisher for String messages."""
        print(f"Creating publisher for topic {self.topic} ...")
        self.publisher = interface.GetNode().create_publisher(String, self.topic, 1)
        return True

    def Tick(self, time: float):
        """Publish concatenated string message."""
        print(f"Publishing {self.ticker} ...")
        msg = String()  # Use String message type
        msg.data = self.message + str(self.ticker)  # Concatenate message and ticker
        self.publisher.publish(msg)
        self.ticker += 1

def main():
    sys = ch.ChSystemNSC()
    sys.SetGravitationalAcceleration(ch.ChVectorD(0, 0, -9.81))  # Corrected vector class
    
    # Define contact material properties
    phys_mat = ch.ChContactMaterialNSC()
    phys_mat.SetFriction(0.5)

    # Create floor
    floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
    floor.SetPos(ch.ChVectorD(0, 0, -1))
    floor.SetFixed(True)
    floor.SetName("base_link")
    sys.Add(floor)

    # Create box
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVectorD(0, 0, 5))
    box.SetRot(ch.Q_from_AngAxis(0.2, ch.ChVectorD(1, 0, 0)))  # Corrected rotation function and vector
    box.SetName("box")
    sys.Add(box)

    # Initialize ROS manager
    ros_manager = chros.ChROSPythonManager()
    
    # Register handlers
    ros_manager.RegisterHandler(chros.ChROSClockHandler())  # Clock handler
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/box"))  # Body handler
    tf_handler = chros.ChROSTFHandler(30)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)  # Transform handler
    
    # Custom handler registration
    custom_handler = MyCustomHandler("~/my_topic")
    ros_manager.RegisterHandler(custom_handler)  # Corrected registration method
    
    # Initialize ROS manager with the system
    ros_manager.Initialize(sys)

    # Simulation parameters
    time_step = 1e-3
    time_end = 30
    realtime_timer = ch.ChRealtimeStepTimer()

    while sys.GetChTime() < time_end:
        sys.DoStepDynamics(time_step)
        current_time = sys.GetChTime()
        
        # Update ROS handlers
        if not ros_manager.Update(current_time, time_step):
            break
        
        realtime_timer.Spin(time_step)

if __name__ == "__main__":
    main()