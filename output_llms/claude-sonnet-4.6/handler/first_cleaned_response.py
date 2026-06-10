import pychrono as chrono
import pychrono.ros as chros
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time




class MyCustomHandler(chros.ChROSHandler):
    
    
    def __init__(self, topic: str, publish_rate: float = 10.0):
        super().__init__(publish_rate)  
        self.topic = topic
        self.publisher = None
        self.message_count = 0
    
    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        
        print(f"Initializing custom handler. Publishing to topic: {self.topic}")
        
        
        self.publisher = interface.GetNode().create_publisher(
            Int32, 
            self.topic, 
            qos_profile=10
        )
        
        return True
    
    def Tick(self, time: float):
        
        msg = Int32()
        msg.data = self.message_count
        
        self.publisher.publish(msg)
        print(f"[t={time:.3f}s] Published integer message: {self.message_count}")
        
        self.message_count += 1





def main():
    
    
    
    sys = chrono.ChSystemNSC()
    
    
    sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    
    
    
    
    phys_material = chrono.ChMaterialSurfaceNSC()
    phys_material.SetFriction(0.5)          
    phys_material.SetRestitution(0.1)       
    phys_material.SetCompliance(0.0)        
    
    
    
    
    floor_body = chrono.ChBodyEasyBox(
        10.0,   
        0.5,    
        10.0,   
        1000.0, 
        True,   
        True,   
        phys_material
    )
    
    
    floor_body.SetPos(chrono.ChVectorD(0, -0.25, 0))
    
    
    floor_body.SetBodyFixed(True)
    floor_body.SetName("floor")
    
    
    floor_vis = chrono.ChColorAsset()
    floor_vis.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
    floor_body.AddAsset(floor_vis)
    
    
    sys.Add(floor_body)
    print("Floor added to the system.")
    
    
    
    
    box_body = chrono.ChBodyEasyBox(
        1.0,    
        1.0,    
        1.0,    
        500.0,  
        True,   
        True,   
        phys_material
    )
    
    
    box_body.SetPos(chrono.ChVectorD(0, 2.0, 0))
    
    
    box_body.SetPos_dt(chrono.ChVectorD(0.5, 0, 0))
    
    
    box_body.SetBodyFixed(False)
    box_body.SetName("box")
    
    
    box_vis = chrono.ChColorAsset()
    box_vis.SetColor(chrono.ChColor(0.2, 0.4, 0.8))
    box_body.AddAsset(box_vis)
    
    
    sys.Add(box_body)
    print("Movable box added to the system.")
    
    
    
    
    ros_manager = chros.ChROSPythonManager()
    
    
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    
    
    
    ros_manager.RegisterHandler(
        chros.ChROSBodyHandler(25, box_body, "/box/state")
    )
    
    
    
    ros_manager.RegisterHandler(
        chros.ChROSTFHandler(25, box_body, "world", "box")
    )
    
    
    ros_manager.RegisterHandler(
        chros.ChROSTFHandler(25, floor_body, "world", "floor")
    )
    
    
    custom_handler = MyCustomHandler("/simulation/counter", publish_rate=5.0)
    ros_manager.RegisterHandler(custom_handler)
    
    print("ROS handlers registered.")
    
    
    ros_manager.Initialize()
    print("ROS manager initialized.")
    
    
    
    
    time_step = 0.001          
    time_end = 10.0            
    sim_time = 0.0
    
    
    realtime_timer = chrono.ChRealtimeStepTimer()
    
    print("\n--- Starting Simulation Loop ---")
    print(f"Simulation will run for {time_end} seconds with dt={time_step}s\n")
    
    step_number = 0
    
    while sim_time <= time_end:
        
        sys.DoStepDynamics(time_step)
        
        
        if not ros_manager.Update(sim_time, time_step):
            print("ROS manager update failed. Stopping simulation.")
            break
        
        
        if step_number % 100 == 0:
            pos = box_body.GetPos()
            print(f"[Step {step_number:5d}] t={sim_time:.3f}s | "
                  f"Box position: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
        
        
        realtime_timer.Spin(time_step)
        
        
        sim_time += time_step
        step_number += 1
    
    print("\n--- Simulation Complete ---")
    print(f"Total steps executed: {step_number}")
    print(f"Final box position: {box_body.GetPos()}")





if __name__ == "__main__":
    main()