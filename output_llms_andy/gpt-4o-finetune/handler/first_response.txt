import pychrono as chrono
import pychrono.robot as chrrobot
import pychrono.ros as chros
import numpy as np

def main():
    # Initialize the Chrono simulation system
    # ---------------------------------------
    # Create a Chrono physical system object to handle simulation dynamics
    # Define a shared material with specific friction properties for object interactions
    sys = chrono.ChSystemNSC()
    mat = chrono.ChContactMaterialNSC()
    mat.SetFriction(0.5)  # Set friction for contact material
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.GetSettings().collision.collision_envelope = 0.01  # Set collision envelope thickness
    sys.GetSettings().collision.narrowphase_algorithm = (
        chrono.ChNarrowPhaseCollider.Algorithm_MANIFOLD  # Use manifold algorithm for collision detection
    )
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Set gravitational acceleration

    # Create a floor as a box object for the simulation
    # ---------------------------------------------------
    # Define the floor box with specific dimensions and position
    # Set the floor as an immovable object with collision properties
    # Add visual representation to the floor
    # Add the floor to the simulation system
    floor_body = chrono.ChBodyEasyBox(
        20, 20, 1,  # Dimensions of the floor box (length, width, height)
        1000,       # Density of the material (not particularly relevant for static objects)
        True,       # Enable visualization
        True,       # Enable collision detection
        mat         # Use the defined contact material
    )
    floor_body.SetPos(chrono.ChVector3d(0, 0, -1))  # Position the floor slightly below the origin
    floor_body.SetFixed(True)  # Make the floor immovable
    floor_body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # Set texture for visualization
    sys.Add(floor_body)  # Add the floor to the simulation system

    # Create a box object for interaction
    # -----------------------------------
    # Define a box with specific dimensions and physical properties
    # Set the initial position of the box above the floor
    # Add visual representation to the box
    # Add the box to the simulation system
    box = chrono.ChBodyEasyBox(
        1, 1, 1,  # Dimensions of the box (length, width, height)
        1000,     # Density of the material
        True,     # Enable visualization
        True,     # Enable collision detection
        mat       # Use the defined contact material
    )
    box.SetPos(chrono.ChVector3d(0, 0, 2))  # Position the box above the floor
    box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))  # Set texture for visualization
    sys.Add(box)  # Add the box to the simulation system

    # Initialize the ROS manager for simulation communication
    # -------------------------------------------------------
    # Create a ROS manager to handle data exchange between Chrono and ROS
    # Register handlers for managing simulation clock, bodies, and transforms in ROS
    ros_manager = chros.ChROSPythonManager()

    # Define a custom ROS handler to publish messages
    # -----------------------------------------------
    # Create and register a custom handler for publishing integer messages to a ROS topic
    class MyCustomHandler(chros.ChROS2CustomHandler):
        """A custom handler example to publish integer messages to a ROS topic."""

        def __init__(self, topic_name):
            super().__init__(
                1
            )  # Initialize the handler with a specific publishing rate (1 Hz)
            self.topic_name = topic_name
            self.publisher = None  # Placeholder for the ROS publisher
            self.msg = chrono.ChInt32()  # Message object to hold the integer data
            self.count = 0  # Counter to keep track of published messages

        def OnInitialize(self):
            """Initialize the ROS publisher."""
            # Create a publisher for the specified ROS topic
            self.publisher = self.GetNode().CreatePublisher(
                self.topic_name, chrono.ChInt32.GetMaxStampedMessageType()
            )

        def OnUpdate(self, time):
            """Publish the integer message to the ROS topic."""
            self.msg.data = self.count  # Set the message data to the current count
            self.publisher.Publish(self.msg)  # Publish the message
            print(f"Publishing: {self.count}")  # Print the published count
            self.count += 1  # Increment the counter for the next message

    # Register standard and custom handlers with the ROS manager
    # -----------------------------------------------------------
    custom_handler = MyCustomHandler("my_topic")
    ros_manager.RegisterHandler(chros.ChROSClockHandler())  # Register clock handler
    ros_manager.RegisterHandler(
        chros.ChROSBodyHandler(sys.GetChTime())
    )  # Register body handler with current simulation time
    ros_manager.RegisterHandler(
        chros.ChROSTFHandler(sys.GetChTime())
    )  # Register transform handler with current simulation time
    ros_manager.RegisterHandler(
        custom_handler
    )  # Register the custom handler for publishing messages
    ros_manager.Initialize()  # Initialize the ROS manager and all registered handlers

    # Run the simulation loop with ROS updates
    # ----------------------------------------
    # Define the simulation time step
    # Initialize counter variables for simulation time tracking
    # Start the simulation loop to advance dynamics and update ROS
    time_step = 1e-3  # Define the time step for the simulation
    time = 0  # Initialize time counter
    time_out = 0  # Initialize output time counter
    time_acc = 0  # Initialize accumulated time counter

    # Simulation loop
    # ---------------
    # Advance the simulation in fixed time steps
    # Update ROS manager to handle communication and data publishing
    # Ensure the loop runs in real-time by dynamically adjusting sleep duration
    max_time = 20  # Set the maximum simulation time
    time_end = sys.GetChTime() + max_time  # Calculate the end time for the simulation loop
    steps = int((time_end - sys.GetChTime()) / time_step)  # Determine the number of steps based on time step

    for i in range(steps):
        sys.DoStepDynamics(time_step)  # Advance the simulation by one time step
        if not ros_manager.Update():
            break  # Exit loop if ROS manager update fails
        time = sys.GetChTime() - time_out  # Update time counter
        time_acc += time  # Accumulate time
        # Dynamically adjust sleep time to maintain real-time loop execution
        time_sleep = max(0, (i + 1) * time_step - time_acc)
        chrono.ChTime().Sleep(time_sleep)  # Sleep for the calculated duration

if __name__ == "__main__":
    main()