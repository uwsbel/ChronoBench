import pychrono as chrono
import pychrono.ros as chronoros


import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32 

import time
import math




class MyCustomIntPublisher(chronoros.ChROSHandler, Node):
    def __init__(self, update_rate, topic_name="custom_int_topic"):
        
        chronoros.ChROSHandler.__init__(self, update_rate)
        
        Node.__init__(self, 'chrono_custom_int_publisher_node')

        self.topic_name = topic_name
        self.publisher_ = self.create_publisher(Int32, self.topic_name, 10)
        self.counter = 0
        self.get_logger().info(f"CustomIntPublisher initialized. Publishing to '{self.topic_name}' at {update_rate} Hz.")

    def Initialize(self, system):
        
        
        super().Initialize(system) 
        self.get_logger().info("CustomIntPublisher's Initialize method called by ChROSManager.")
        

    def Update(self, time_chrono, time_ros):
        
        if not self.publisher_:
            self.get_logger().warn("Publisher not initialized!")
            return

        msg = Int32()
        msg.data = self.counter
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing to {self.topic_name}: "{msg.data}" (Chrono time: {time_chrono:.2f}s)')
        self.counter += 1




def main():
    print("Initializing PyChrono-ROS simulation...")

    
    
    
    
    rclpy.init()

    
    my_system = chrono.ChSystemNSC() 
    my_system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    
    my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN) 
    my_system.GetSolver().AsIterative().SetMaxIterations(50)
    my_system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


    
    contact_material = chrono.ChContactMaterialNSC()
    contact_material.SetFriction(0.4)
    contact_material.SetRestitution(0.1) 

    
    

    
    body_floor = chrono.ChBody()
    body_floor.SetFixed(True)
    body_floor.SetName("floor") 
    body_floor.SetPos(chrono.ChVector3d(0, -1, 0)) 

    
    floor_coll_shape = chrono.ChCollisionShapeBox(contact_material, 20, 2, 20) 
    body_floor.AddCollisionShape(floor_coll_shape)

    
    floor_vis_shape = chrono.ChVisualShapeBox(20, 2, 20)
    floor_vis_shape.SetColor(chrono.ChColor(0.3, 0.3, 0.5))
    body_floor.AddVisualShape(floor_vis_shape)

    my_system.Add(body_floor)

    
    box_mass = 10.0
    box_size = chrono.ChVector3d(1, 1, 1) 
    box_half_size = box_size / 2.0

    body_box = chrono.ChBody()
    body_box.SetMass(box_mass)
    body_box.SetInertiaXX(chrono.ChVector3d(
        (1.0/12.0) * box_mass * (box_size.y**2 + box_size.z**2),
        (1.0/12.0) * box_mass * (box_size.x**2 + box_size.z**2),
        (1.0/12.0) * box_mass * (box_size.x**2 + box_size.y**2)
    ))
    body_box.SetName("movable_box")
    body_box.SetPos(chrono.ChVector3d(0, 2, 0)) 
    body_box.SetRot(chrono.QuatFromAngleAxis(math.pi / 6, chrono.ChVector3d(0,0,1))) 

    
    box_coll_shape = chrono.ChCollisionShapeBox(contact_material, box_half_size.x, box_half_size.y, box_half_size.z)
    body_box.AddCollisionShape(box_coll_shape)

    
    box_vis_shape = chrono.ChVisualShapeBox(box_size.x, box_size.y, box_size.z)
    box_vis_shape.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
    body_box.AddVisualShape(box_vis_shape)

    my_system.Add(body_box)


    
    
    ros_manager_update_rate = 100 
    ros_manager = chronoros.ChROSManager()
    ros_manager.SetUpdateRate(ros_manager_update_rate)

    
    clock_handler = chronoros.ChROSClockHandler()
    ros_manager.RegisterHandler(clock_handler)

    
    
    body_handler_update_rate = 30 
    body_handler = chronoros.ChROSBodyHandler(body_handler_update_rate, my_system)
    
    
    
    body_handler.AddBody(body_box, "tracked_box_pose") 
    ros_manager.RegisterHandler(body_handler)

    
    
    
    tf_handler_update_rate = 30 
    tf_handler = chronoros.ChROSTransformHandler(tf_handler_update_rate, my_system)
    ros_manager.RegisterHandler(tf_handler)

    
    custom_handler_update_rate = 2 
    custom_int_handler = MyCustomIntPublisher(custom_handler_update_rate, "chrono_sim_counter")
    ros_manager.RegisterHandler(custom_int_handler)

    
    print("Initializing ROS Manager and handlers...")
    ros_manager.Initialize()
    print("ROS Manager initialized.")


    
    
    timestep = 0.005  
    simulation_duration = 20.0 
    current_time = 0.0

    
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
            
            my_system.DoStepDynamics(timestep)
            current_time = my_system.GetChTime()

            
            
            
            ros_manager.Update(current_time, timestep)


            
            
            
            
            
            rclpy.spin_once(ros_manager.GetNode(), timeout_sec=0.0001) 
            rclpy.spin_once(custom_int_handler, timeout_sec=0.0001) 

            
            realtime_timer.Spin(timestep)

            if int(current_time * 100) % 100 == 0: 
                 print(f"Sim Time: {current_time:.2f} s, Box Z: {body_box.GetPos().z:.3f} m")

    except KeyboardInterrupt:
        print("Simulation interrupted by user.")
    finally:
        print("Shutting down ROS and Chrono...")
        ros_manager.Shutdown() 
        rclpy.shutdown()       
        print("Simulation finished.")

if __name__ == '__main__':
    main()