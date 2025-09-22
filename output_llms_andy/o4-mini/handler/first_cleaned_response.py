import time


import pychrono.core as chrono
import pychrono.ros as chrono_ros


import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class CustomIntPublisher(chrono_ros.ChRosMsgHandler):
    

    def __init__(self, node: Node, topic: str):
        super().__init__()
        self.node = node
        self.publisher = node.create_publisher(Int32, topic, 10)
        self.counter = 0

    def OnPublish(self, current_time: float):
        
        msg = Int32()
        
        msg.data = self.counter
        self.publisher.publish(msg)
        self.counter += 1


def main():
    
    rclpy.init()
    ros_node = rclpy.create_node('pychrono_ros2_sim')

    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

    
    mat = chrono.ChMaterialSurfaceNSC()

    
    floor = chrono.ChBody()
    floor.SetName("ground")
    floor.SetBodyFixed(True)
    floor.SetMaterialSurface(mat)
    
    floor_shape = chrono.ChBoxShape()
    floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(5, 5, 0.1)
    floor.AddVisualShape(floor_shape)
    floor.SetPos(chrono.ChVectorD(0, 0, -0.1))
    system.Add(floor)

    
    box = chrono.ChBodyEasyBox(
        1.0,    
        1.0,    
        1.0,    
        1000,   
        True,   
        True    
    )
    box.SetName("box")
    box.SetMaterialSurface(mat)
    box.SetPos(chrono.ChVectorD(0, 0, 1.0))
    system.Add(box)

    
    
    manager = chrono_ros.ChRosManager.Create(ros_node)
    manager.AddSystem(system)             
    manager.AddClockPublisher()           
    manager.AddBodyPublisher(box, "box")  
    manager.AddTFPublisher()              

    
    custom_int = CustomIntPublisher(ros_node, "my_custom_int")
    manager.RegisterHandler(custom_int)

    
    dt = 1e-3   
    rate = ros_node.create_rate(1.0 / dt)  

    
    while rclpy.ok():
        system.DoStepDynamics(dt)

        
        manager.OnAdvance(system.GetChTime())

        
        rclpy.spin_once(ros_node, timeout_sec=0)

        
        rate.sleep()

    
    rclpy.shutdown()


if __name__ == "__main__":
    main()