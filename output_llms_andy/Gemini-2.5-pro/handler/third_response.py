import pychrono as ch
import pychrono.irrlicht as irr # Added for Irrlicht visualization
import pychrono.ros as chros

# rclpy.publisher is used for type hinting, ensure rclpy is installed
import rclpy.publisher 
from std_msgs.msg import Int64

# Attempt to set Chrono data directory (important for textures)
# User might need to change this path to their Chrono data installation
try:
    # This uses the CHRONO_DATA_DIR environment variable if already set.
    # If not set, PyChrono might try a default path.
    # For explicit path setting, use: ch.SetChronoDataPath('path/to/your/chrono/data/')
    chrono_data_path = ch.GetChronoDataPath()
    if chrono_data_path: # If a path is found, try to set it (might be redundant but harmless)
        ch.SetChronoDataPath(chrono_data_path)
    else: # If GetChronoDataPath returns empty (e.g. env var not set)
        print("Warning: CHRONO_DATA_DIR environment variable not found or Chrono data path not set.")
        print("Attempting to use a default relative path './chrono_data/' for data.")
        print("If textures/logo are not found, please set CHRONO_DATA_DIR or use ch.SetChronoDataPath().")
        # As a fallback, you might try a common relative path if you know where data is.
        # For this example, we'll rely on GetChronoDataFile to search default locations.
except Exception as e:
    print(f"Warning: Could not decisively set Chrono data path. Textures might not load. {e}")


# Define a custom ROS handler for publishing integer messages.
class MyCustomHandler(chros.ChROSHandler):
    """This custom handler will publish integer messages to a specified ROS topic."""

    # Modified __init__ to accept update_rate
    def __init__(self, update_rate: float, topic: str):
        super().__init__(update_rate)  # Use the provided update_rate

        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None # Type hint for clarity
        self.ticker = 0  # Initialize a counter for published messages.

    # Corrected type hint for interface to ChROSPythonManager
    def Initialize(self, interface: chros.ChROSPythonManager) -> bool:
        """Initialize the ROS publisher."""
        print(f"MyCustomHandler: Creating publisher for topic {self.topic} ...")
        # Create a ROS publisher for the specified topic.
        # The node is obtained from the manager instance (interface)
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 10) # Queue size 10
        return True  # Return True to indicate successful initialization.

    def Tick(self, time: float):
        """Publish an integer message to the ROS topic."""
        # This print can be verbose, consider removing for long runs or reducing frequency
        # print(f"MyCustomHandler: Publishing {self.ticker} at time {time:.2f} to {self.topic} ...")
        msg = Int64()  # Create a message object of type Int64.
        msg.data = self.ticker  # Set the message data to the current ticker value.
        self.publisher.publish(msg)  # Publish the message to the ROS topic.
        self.ticker += 1  # Increment the ticker for the next message.

def main():
    # Create the Chrono simulation system.
    sys = ch.ChSystemNSC()
    sys.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.81))  # Set gravitational acceleration.

    # Define physical material properties for contact.
    phys_mat = ch.ChContactMaterialNSC()
    phys_mat.SetFriction(0.5)  # Set friction coefficient.

    # Create a floor object.
    # Thickness is 1, so if center is at z=-0.5, top surface is at z=0.
    floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
    floor.SetPos(ch.ChVector3d(0, 0, -0.5))
    floor.SetFixed(True)  # Fix the floor in place.
    floor.SetName("base_link")  # Set the name for ROS communication.
    sys.Add(floor)  # Add the floor to the simulation system.

    # Create a box object.
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 2))  # Position the box above the floor.
    box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))  # Rotate the box slightly.
    box.SetName("box")  # Set the name for ROS communication.
    sys.Add(box)  # Add the box to the simulation system.

    # Set textures for the floor and box
    try:
        floor_texture_path = ch.GetChronoDataFile('textures/concrete.jpg')
        box_texture_path = ch.GetChronoDataFile('textures/bluewhite.png')
        
        floor_vis_shape = floor.GetVisualShape(0) 
        if floor_vis_shape:
             floor_vis_shape.SetTexture(floor_texture_path)

        box_vis_shape = box.GetVisualShape(0)
        if box_vis_shape:
            box_vis_shape.SetTexture(box_texture_path)
            
    except Exception as e:
        print(f"Error setting textures: {e}. Make sure Chrono data path is correct and textures exist.")

    # Create and configure the ROS manager.
    ros_manager = chros.ChROSPythonManager()
    
    # Define ROS publish rate
    publish_rate = 10.0  # Hz

    # Register a clock handler for the simulation time.
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    
    # Register a body handler to communicate the box's state.
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(publish_rate, box, "~/box_state"))
    
    # Create and register a transform handler for coordinate transformations.
    tf_handler = chros.ChROSTFHandler(publish_rate)
    # Publish floor pose relative to the Chrono world frame (e.g. "/chrono_world" -> "base_link")
    tf_handler.AddTransform(floor, floor.GetName()) 
    # Publish box pose relative to the floor (e.g. "base_link" -> "box")
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)
    
    # Register the custom handler to publish messages.
    custom_handler = MyCustomHandler(publish_rate, "~/my_topic_data")
    ros_manager.RegisterPythonHandler(custom_handler)

    # Initialize the ROS manager (this also initializes rclpy and creates the node).
    ros_manager.Initialize()

    # Setup Irrlicht visualization
    myapplication = irr.ChVisualSystemIrrlicht()
    myapplication.AttachSystem(sys)
    myapplication.SetWindowTitle('PyChrono ROS Simulation with Irrlicht')
    myapplication.SetWindowSize(1024, 768)
    myapplication.Initialize()
    try:
        myapplication.AddLogo(ch.GetChronoDataFile('logo_pychrono_alpha.png'))
    except Exception as e:
        print(f"Could not load logo: {e}")
    myapplication.AddSkyBox()
    myapplication.AddTypicalLights()
    myapplication.AddCamera(ch.ChVector3d(3, -4, 3), ch.ChVector3d(0, 0, 0.5)) # Adjusted camera
    myapplication.BindAll() # Bind all visual assets


    # Run the simulation loop.
    time = 0.0
    time_step = 1e-3  # Define the simulation time step.
    time_end = 30.0  # Set the duration for the simulation.

    # Variables for controlling rendering frame rate
    step_number = 0
    render_step_size = time_step # Smallest unit of time for rendering control
    # Render roughly at 25 FPS if time_step = 1e-3s: (1/25s) / 0.001s/step = 40 steps
    render_steps = 40 

    realtime_timer = ch.ChRealtimeStepTimer()
    
    print("Starting simulation loop...")
    while myapplication.GetDevice().run() and time < time_end:
        # Advance the simulation by one time step.
        sys.DoStepDynamics(time_step)
        time = sys.GetChTime()

        # Update ROS communication.
        if not ros
print("error happened with only start ```python")