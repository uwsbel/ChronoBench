# PyChrono imports
import pychrono as chrono
import pychrono.ros as chronoros

# ROS specific imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32 # For our custom integer message

import time
import math

# -----------------------------------------------------------------------------
# Custom ROS Handler to publish integer messages
# -----------------------------------------------------------------------------
class MyCustomIntPublisher(chronoros.ChROSHandler, Node):
    def __init__(self, update_rate, topic_name="custom_int_topic"):
        # Initialize ChROSHandler first
        chronoros.ChROSHandler.__init__(self, update_rate)
        # Initialize rclpy.node.Node
        Node.__init__(self, 'chrono_custom_int_publisher_node')

        self.topic_name = topic_name
        self.publisher_ = self.create_publisher(Int32, self.topic_name, 10)
        self.counter = 0
        self.get_logger().info(f"CustomIntPublisher initialized. Publishing to '{self.topic_name}' at {update_rate} Hz.")

    def Initialize(self, system):
        # This method is called by ChROSManager when the handler is registered
        # and ChROSManager.Initialize() is called.
        super().Initialize(system) # Call base class Initialize
        self.get_logger().info("CustomIntPublisher's Initialize method called by ChROSManager.")
        # You could do system-dependent initialization here if needed

    def Update(self, time_chrono, time_ros):
        # This method is called by ChROSManager.Update() at the specified update_rate
        if not self.publisher_:
            self.get_logger().warn("Publisher not initialized!")
            return

        msg = Int32()
        msg.data = self.counter
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing to {self.topic_name}: "{msg.data}" (Chrono time: {time_chrono:.2f}s)')
        self.counter += 1

# -----------------------------------------------------------------------------
# Main simulation script
# -----------------------------------------------------------------------------
def main():
    print("Initializing PyChrono-ROS simulation...")

    # 1. Initialize PyChrono environment and core components
    # -----------------------------------------------------
    # Initialize rclpy (ROS Python client library)
    # This should be done before creating any ROS nodes or the ChROSManager
    rclpy.init()

    # Create a Chrono system
    my_system = chrono.ChSystemNSC() # Use Non-Smooth Contact solver
    my_system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    # Set solver settings (optional, but good practice)
    my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN) # A fast solver
    my_system.GetSolver().AsIterative().SetMaxIterations(50)
    my_system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


    # Define a physical material
    contact_material = chrono.ChContactMaterialNSC()
    contact_material.SetFriction(0.4)
    contact_material.SetRestitution(0.1) # Bounciness

    # 2. Add required physical systems and objects
    # --------------------------------------------

    # Add a fixed floor
    body_floor = chrono.ChBody()
    body_floor.SetFixed(True)
    body_floor.SetName("floor") # Naming is good practice for ROS handlers
    body_floor.SetPos(chrono.ChVector3d(0, -1, 0)) # Position the center of the floor box

    # Collision shape for the floor
    floor_coll_shape = chrono.ChCollisionShapeBox(contact_material, 20, 2, 20) # material, half-lengths dx, dy, dz
    body_floor.AddCollisionShape(floor_coll_shape)

    # Visualization shape for the floor
    floor_vis_shape = chrono.ChVisualShapeBox(20, 2, 20)
    floor_vis_shape.SetColor(chrono.ChColor(0.3, 0.3, 0.5))
    body_floor.AddVisualShape(floor_vis_shape)

    my_system.Add(body_floor)

    # Add a movable box
    box_mass = 10.0
    box_size = chrono.ChVector3d(1, 1, 1) # Full lengths dx, dy, dz
    box_half_size = box_size / 2.0

    body_box = chrono.ChBody()
    body_box.SetMass(box_mass)
    body_box.SetInertiaXX(chrono.ChVector3d(
        (1.0/12.0) * box_mass * (box_size.y**2 + box_size.z**2),
        (1.0/12.0) * box_mass * (box_size.x**2 + box_size.z**2),
        (1.0/12.0) * box_mass * (box_size.x**2 + box_size.y**2)
    ))
    body_box.SetName("movable_box")
    body_box.SetPos(chrono.ChVector3d(0, 2, 0)) # Initial position above the floor
    body_box.SetRot(chrono.QuatFromAngleAxis(math.pi / 6, chrono.ChVector3d(0,0,1))) # Initial rotation

    # Collision shape for the box
    box_coll_shape = chrono.ChCollisionShapeBox(contact_material, box_half_size.x, box_half_size.y, box_half_size.z)
    body_box.AddCollisionShape(box_coll_shape)

    # Visualization shape for the box
    box_vis_shape = chrono.ChVisualShapeBox(box_size.x, box_size.y, box_size.z)
    box_vis_shape.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
    body_box.AddVisualShape(box_vis_shape)

    my_system.Add(body_box)


    # 3. Configure the ROS manager and handlers
    # ------------------------------------------
    ros_manager_update_rate = 100 # Hz - How often ROS manager updates its internal state and calls handler Updates
    ros_manager = chronoros.ChROSManager()
    ros_manager.SetUpdateRate(ros_manager_update_rate)

    # Clock handler (publishes /clock)
    clock_handler = chronoros.ChROSClockHandler()
    ros_manager.RegisterHandler(clock_handler)

    # Body handler (publishes poses of named bodies)
    # The update rate here is for the specific handler, not the manager's overall rate
    body_handler_update_rate = 30 # Hz
    body_handler = chronoros.ChROSBodyHandler(body_handler_update_rate, my_system)
    # To publish specific bodies, you can add them by name.
    # If not specified, it might try to publish all named bodies.
    # Let's explicitly add the box to be tracked
    body_handler.AddBody(body_box, "tracked_box_pose") # Topic will be /chrono_ros/tracked_box_pose
    ros_manager.RegisterHandler(body_handler)

    # Transform handler (publishes TF data)
    # This will publish transforms for named bodies if their frames are set up.
    # By default, it publishes body frames relative to the world frame.
    tf_handler_update_rate = 30 # Hz
    tf_handler = chronoros.ChROSTransformHandler(tf_handler_update_rate, my_system)
    ros_manager.RegisterHandler(tf_handler)

    # Custom integer publisher handler
    custom_handler_update_rate = 2 # Hz (Publish integer messages at 2 Hz)
    custom_int_handler = MyCustomIntPublisher(custom_handler_update_rate, "chrono_sim_counter")
    ros_manager.RegisterHandler(custom_int_handler)

    # Initialize the ROS manager (this also initializes all registered handlers)
    print("Initializing ROS Manager and handlers...")
    ros_manager.Initialize()
    print("ROS Manager initialized.")


    # 4. Run the simulation loop
    # --------------------------
    timestep = 0.005  # Simulation timestep
    simulation_duration = 20.0 # seconds
    current_time = 0.0

    # For real-time execution (approximate)
    realtime_timer = chrono.ChRealtimeStepTimer()

    print(f"\nStarting simulation loop for {simulation_duration} seconds...")
    print(f"Chrono timestep: {timestep} s")
    print(f"ROS Manager update rate: {ros_manager_update_rate} Hz")
    print(f"Custom Int Publisher update rate: {custom_handler_update_rate} Hz")
    print("----------------------------------------------------")
    print("You should be able to see topics like:")
    print("  /clock")
    print("  /tf and /tf_static (for floor and movable_box)")
    print("  /chrono_ros/tracked_box_pose (geometry_msgs/PoseStamped)")
    print(f"  /{custom_int_handler.topic_name} (std_msgs/Int32)")
    print("using 'ros2 topic list' and 'ros2 topic echo <topic_name>'")
    print("----------------------------------------------------\n")


    try:
        while current_time < simulation_duration and rclpy.ok():
            # Advance Chrono system
            my_system.DoStepDynamics(timestep)
            current_time = my_system.GetChTime()

            # Update ROS communication
            # The first argument is Chrono time, second is wall time (or can be timestep)
            # ros_manager.Update(current_time, my_system.GetStep()) # Using GetStep() as dt_wall
            ros_manager.Update(current_time, timestep)


            # Spin ROS services to allow callbacks to be processed
            # We need to spin our custom handler node specifically if it has subscriptions
            # or services. For publishers, spinning is good practice to ensure messages
            # are flushed and internal ROS processing happens.
            # ChROSManager also has an internal node that benefits from spinning.
            rclpy.spin_once(ros_manager.GetNode(), timeout_sec=0.0001) # Spin manager's node
            rclpy.spin_once(custom_int_handler, timeout_sec=0.0001) # Spin custom handler's node

            # Maintain real-time execution (optional)
            realtime_timer.Spin(timestep)

            if int(current_time * 100) % 100 == 0: # Print roughly every second
                 print(f"Sim Time: {current_time:.2f} s, Box Z: {body_box.GetPos().z:.3f} m")

    except KeyboardInterrupt:
        print("Simulation interrupted by user.")
    finally:
        print("Shutting down ROS and Chrono...")
        ros_manager.Shutdown() # Properly shut down ROS handlers
        rclpy.shutdown()       # Shut down rclpy
        print("Simulation finished.")

if __name__ == '__main__':
    main()