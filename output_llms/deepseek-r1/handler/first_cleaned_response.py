import pychrono as chrono
import pychrono.ros as chros
import numpy as np

def main():
    
    chros.NodeInit("pychrono_sim")

    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

    
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(0.4)
    material.SetCompliance(1e-5)

    
    floor = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True, material)
    floor.SetPos(chrono.ChVectorD(0, -0.5, 0))
    floor.SetBodyFixed(True)
    floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    system.Add(floor)

    
    box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, material)
    box.SetPos(chrono.ChVectorD(0, 2, 0))
    box.GetVisualShape(0).SetColor(chrono.ChColor(0.6, 0.2, 0.2))
    system.Add(box)

    
    ros_manager = chros.ChROSParams()
    ros_manager.handle_clock = True
    ros_manager.handle_sensors = False
    ros_manager.handle_visualization = False
    manager = chros.ChROS(system, ros_manager)

    
    class IntPublisher(chros.ChROSHandler):
        def __init__(self):
            super().__init__()
            self.topic = "/simulation/step_count"
            self.publisher = None
            self.count = 0

        def Initialize(self):
            self.publisher = chros.Node().create_publisher(chros.Int64, self.topic, 1)
            return True

        def Tick(self, time):
            msg = chros.Int64()
            msg.data = self.count
            self.publisher.publish(msg)
            self.count += 1
            return True

    
    manager.RegisterHandler(chros.ChROSClockHandler())  
    manager.RegisterHandler(chros.ChROSBodyHandler(system))  
    manager.RegisterHandler(chros.ChROSTFHandler(system, 1))  
    manager.RegisterHandler(IntPublisher())  

    
    if not manager.Initialize():
        print("ROS initialization failed!")
        return

    
    time_step = 0.01
    realtime_step = True
    frame_time = 0.0

    while manager.NodeOK():
        
        system.DoStepDynamics(time_step)
        
        
        manager.Advance(time_step)
        
        
        if realtime_step:
            frame_time += time_step
            manager.SpinSome()
            manager.Sleep(frame_time)

if __name__ == "__main__":
    main()