import pychrono as chrono
import pychrono.ros as chronoros
import rclpy
from std_msgs.msg import Int32
import time
import math

class CustomIntegerHandler(chronoros.ChROSHandler):
    
    
    def __init__(self, topic_name):
        super().__init__(1.0/10.0)  
        self.topic_name = topic_name
        self.counter = 0
        
    def Initialize(self, node):
        
        self.publisher = node.create_publisher(Int32, self.topic_name, 10)
        print(f"CustomIntegerHandler initialized for topic: {self.topic_name}")
        
    def Tick(self, time):
        
        msg = Int32()
        msg.data = self.counter
        self.publisher.publish(msg)
        print(f"Published integer: {self.counter} at time: {time:.2f}")
        self.counter += 1

def main():
    
    rclpy.init()
    
    
    system = chrono.ChSystemNSC()
    
    
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    
    
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(0.5)
    material.SetRestitution(0.3)
    material.SetRollingFriction(0.01)
    material.SetSpinningFriction(0.01)
    
    
    floor = chrono.ChBodyEasyBox(10, 0.2, 10, 1000, True, True, material)
    floor.SetPos(chrono.ChVectorD(0, -0.1, 0))
    floor.SetBodyFixed(True)
    floor.SetName("floor")
    
    
    floor_asset = floor.GetVisualShape(0)
    if floor_asset:
        floor_asset.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
    
    system.Add(floor)
    
    
    box_size = chrono.ChVectorD(0.5, 0.5, 0.5)
    box = chrono.ChBodyEasyBox(box_size.x, box_size.y, box_size.z, 100, True, True, material)
    box.SetPos(chrono.ChVectorD(0, 2, 0))
    box.SetRot(chrono.Q_from_AngAxis(0.1, chrono.ChVectorD(1, 0, 0)))
    box.SetName("movable_box")
    
    
    box_asset = box.GetVisualShape(0)
    if box_asset:
        box_asset.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
    
    
    box.SetPos_dt(chrono.ChVectorD(0.5, 0, 0))
    box.SetWvel_par(chrono.ChVectorD(0, 0, 0.2))
    
    system.Add(box)
    
    
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.SetSolverMaxIterations(100)
    system.SetMaxPenetrationRecoverySpeed(0.1)
    
    
    ros_manager = chronoros.ChROSPythonManager()
    
    
    clock_handler = chronoros.ChROSClockHandler()
    ros_manager.RegisterHandler(clock_handler)
    
    
    body_handler = chronoros.ChROSBodyHandler(1.0/30.0, box, "movable_box")
    ros_manager.RegisterHandler(body_handler)
    
    
    tf_handler = chronoros.ChROSTFHandler(1.0/30.0)
    tf_handler.AddTransform(box, "movable_box", "world")
    tf_handler.AddTransform(floor, "floor", "world")
    ros_manager.RegisterHandler(tf_handler)
    
    
    custom_handler = CustomIntegerHandler("simulation_counter")
    ros_manager.RegisterHandler(custom_handler)
    
    
    ros_manager.Initialize()
    
    
    timestep = 0.01
    simulation_time = 0
    real_time_factor = 1.0
    
    print("Starting PyChrono-ROS simulation...")
    print(f"Timestep: {timestep}s")
    print(f"Real-time factor: {real_time_factor}")
    
    
    try:
        while rclpy.ok():
            start_time = time.time()
            
            
            system.DoStepDynamics(timestep)
            simulation_time += timestep
            
            
            ros_manager.Update(simulation_time, timestep)
            
            
            if int(simulation_time * 10) % 100 == 0:  
                force = chrono.ChVectorD(50 * math.sin(simulation_time), 
                                       100, 
                                       50 * math.cos(simulation_time))
                box.Empty_forces_accumulators()
                box.Accumulate_force(force, box.GetPos(), False)
            
            
            if int(simulation_time * 100) % 500 == 0:
                box_pos = box.GetPos()
                box_vel = box.GetPos_dt()
                print(f"Time: {simulation_time:.2f}s, Box pos: ({box_pos.x:.2f}, {box_pos.y:.2f}, {box_pos.z:.2f}), "
                      f"Box vel: ({box_vel.x:.2f}, {box_vel.y:.2f}, {box_vel.z:.2f})")
            
            
            elapsed_time = time.time() - start_time
            sleep_time = (timestep / real_time_factor) - elapsed_time
            if sleep_time > 0:
                time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
    
    finally:
        
        print("Cleaning up ROS resources...")
        rclpy.shutdown()
        print("Simulation finished")

if __name__ == "__main__":
    main()