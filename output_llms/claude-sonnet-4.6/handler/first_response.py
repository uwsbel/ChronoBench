import pychrono as chrono
import pychrono.ros as chros
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time

# =============================================================================
# Custom ROS Handler to publish integer messages
# =============================================================================
class MyCustomHandler(chros.ChROSHandler):
    """Custom ROS handler that publishes integer messages to a specified topic."""
    
    def __init__(self, topic: str, publish_rate: float = 10.0):
        super().__init__(publish_rate)  # Set the publish rate in Hz
        self.topic = topic
        self.publisher = None
        self.message_count = 0
    
    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        """Initialize the handler and create a ROS publisher."""
        print(f"Initializing custom handler. Publishing to topic: {self.topic}")
        
        # Create a ROS publisher for Int32 messages
        self.publisher = interface.GetNode().create_publisher(
            Int32, 
            self.topic, 
            qos_profile=10
        )
        
        return True
    
    def Tick(self, time: float):
        """Called at each update step to publish the integer message."""
        msg = Int32()
        msg.data = self.message_count
        
        self.publisher.publish(msg)
        print(f"[t={time:.3f}s] Published integer message: {self.message_count}")
        
        self.message_count += 1


# =============================================================================
# Main simulation setup
# =============================================================================
def main():
    # -------------------------------------------------------------------------
    # 1. Initialize the Chrono system
    # -------------------------------------------------------------------------
    sys = chrono.ChSystemNSC()
    
    # Set gravitational acceleration (standard Earth gravity in -Y direction)
    sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    
    # -------------------------------------------------------------------------
    # 2. Create physical material properties
    # -------------------------------------------------------------------------
    phys_material = chrono.ChMaterialSurfaceNSC()
    phys_material.SetFriction(0.5)          # Coefficient of friction
    phys_material.SetRestitution(0.1)       # Coefficient of restitution
    phys_material.SetCompliance(0.0)        # Compliance (rigid contact)
    
    # -------------------------------------------------------------------------
    # 3. Add a fixed floor to the system
    # -------------------------------------------------------------------------
    floor_body = chrono.ChBodyEasyBox(
        10.0,   # X dimension
        0.5,    # Y dimension (thickness)
        10.0,   # Z dimension
        1000.0, # Density (kg/m^3)
        True,   # Visualization enabled
        True,   # Collision enabled
        phys_material
    )
    
    # Position the floor at the origin (slightly below y=0)
    floor_body.SetPos(chrono.ChVectorD(0, -0.25, 0))
    
    # Fix the floor so it doesn't move
    floor_body.SetBodyFixed(True)
    floor_body.SetName("floor")
    
    # Set floor visualization color
    floor_vis = chrono.ChColorAsset()
    floor_vis.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
    floor_body.AddAsset(floor_vis)
    
    # Add floor to the system
    sys.Add(floor_body)
    print("Floor added to the system.")
    
    # -------------------------------------------------------------------------
    # 4. Add a movable box to the system
    # -------------------------------------------------------------------------
    box_body = chrono.ChBodyEasyBox(
        1.0,    # X dimension
        1.0,    # Y dimension
        1.0,    # Z dimension
        500.0,  # Density (kg/m^3)
        True,   # Visualization enabled
        True,   # Collision enabled
        phys_material
    )
    
    # Position the box above the floor
    box_body.SetPos(chrono.ChVectorD(0, 2.0, 0))
    
    # Give the box an initial velocity (optional)
    box_body.SetPos_dt(chrono.ChVectorD(0.5, 0, 0))
    
    # Set the box as movable (not fixed)
    box_body.SetBodyFixed(False)
    box_body.SetName("box")
    
    # Set box visualization color
    box_vis = chrono.ChColorAsset()
    box_vis.SetColor(chrono.ChColor(0.2, 0.4, 0.8))
    box_body.AddAsset(box_vis)
    
    # Add box to the system
    sys.Add(box_body)
    print("Movable box added to the system.")
    
    # -------------------------------------------------------------------------
    # 5. Set up the ROS manager
    # -------------------------------------------------------------------------
    ros_manager = chros.ChROSPythonManager()
    
    # Register clock handler (publishes simulation time to /clock)
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    
    # Register body handler (publishes body states)
    # Publish the box body state at 25 Hz
    ros_manager.RegisterHandler(
        chros.ChROSBodyHandler(25, box_body, "/box/state")
    )
    
    # Register transform handler (publishes TF transforms)
    # Publish the transform for the box body at 25 Hz
    ros_manager.RegisterHandler(
        chros.ChROSTFHandler(25, box_body, "world", "box")
    )
    
    # Register floor transform handler
    ros_manager.RegisterHandler(
        chros.ChROSTFHandler(25, floor_body, "world", "floor")
    )
    
    # Create and register the custom integer publisher handler
    custom_handler = MyCustomHandler("/simulation/counter", publish_rate=5.0)
    ros_manager.RegisterHandler(custom_handler)
    
    print("ROS handlers registered.")
    
    # Initialize the ROS manager
    ros_manager.Initialize()
    print("ROS manager initialized.")
    
    # -------------------------------------------------------------------------
    # 6. Run the simulation loop
    # -------------------------------------------------------------------------
    time_step = 0.001          # Simulation timestep (1 ms)
    time_end = 10.0            # Total simulation time (seconds)
    sim_time = 0.0
    
    # Real-time timer
    realtime_timer = chrono.ChRealtimeStepTimer()
    
    print("\n--- Starting Simulation Loop ---")
    print(f"Simulation will run for {time_end} seconds with dt={time_step}s\n")
    
    step_number = 0
    
    while sim_time <= time_end:
        # Advance the Chrono simulation by one timestep
        sys.DoStepDynamics(time_step)
        
        # Update ROS communication (publish messages, spin node)
        if not ros_manager.Update(sim_time, time_step):
            print("ROS manager update failed. Stopping simulation.")
            break
        
        # Print box position every 100 steps
        if step_number % 100 == 0:
            pos = box_body.GetPos()
            print(f"[Step {step_number:5d}] t={sim_time:.3f}s | "
                  f"Box position: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
        
        # Maintain real-time execution
        realtime_timer.Spin(time_step)
        
        # Update simulation time and step counter
        sim_time += time_step
        step_number += 1
    
    print("\n--- Simulation Complete ---")
    print(f"Total steps executed: {step_number}")
    print(f"Final box position: {box_body.GetPos()}")


# =============================================================================
# Entry point
# =============================================================================
if __name__ == "__main__":
    main()