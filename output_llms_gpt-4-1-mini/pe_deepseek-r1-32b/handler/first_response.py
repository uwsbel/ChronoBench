import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as chronorobot
import pychrono irrlicht as chronoirr
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

# Initialize ROS node and publisher
class ROSHandler(Node):
    def __init__(self):
        super().__init__('chrono_simulation_node')
        self.publisher_ = self.create_publisher(Int32, 'simulation_time', 10)
        
    def publish_time(self, time):
        msg = Int32()
        msg.data = int(time)
        self.publisher_.publish(msg)

def main():
    # Initialize ROS
    rclpy.init(args=None)
    ros_handler = ROSHandler()
    
    # Initialize Chrono system
    sys = chrono.ChSystemSMC()
    sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity
    
    # Visualization setup
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.SetWindowTitle('ROS-PyChrono Simulation')
    vis.SetWindowSize(800, 600)
    vis.AttachSystem(sys)
    vis.Initialize()
    vis.AddTypicalLights()
    
    # Create fixed floor
    floor = chrono.ChBody()
    floor.SetPos(chrono.ChVectorD(0, -1, 0))
    floor.SetFixed(True)
    floor_shape = chrono.ChVisualShapeBox(chrono.ChVectorD(10, 1, 10))
    floor.AddVisualShape(floor_shape)
    sys.Add(floor)
    
    # Create movable box
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000)
    box.SetPos(chrono.ChVectorD(0, 1, 0))
    box_shape = chrono.ChVisualShapeBox(chrono.ChVectorD(0.5, 0.5, 0.5))
    box.AddVisualShape(box_shape)
    sys.Add(box)
    
    # ROS manager setup
    ros_manager = chronorobot.ChRosManager(sys)
    ros_manager.SetRosDomainID(1000)  # Set ROS domain ID
    ros_manager.RegisterRosClockHandler()
    ros_manager.RegisterRosBodyHandler()
    ros_manager.RegisterRosTransformHandler()
    ros_manager.RegisterRosCustomHandler(ros_handler)
    
    # Simulation parameters
    timestep = 0.001
    realtime_factor = 1.0
    
    # Simulation loop
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        # Advance simulation
        sys.DoStepDynamics(timestep)
        
        # Update ROS handlers
        ros_manager.Update(timestep)
        
        # Publish simulation time
        current_time = sys.GetChTime()
        ros_handler.publish_time(current_time)
        
        # Real-time execution
        if realtime_factor > 0:
            sleep_time = timestep / realtime_factor
            time.sleep(sleep_time)
    
    # Cleanup
    vis.Close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()