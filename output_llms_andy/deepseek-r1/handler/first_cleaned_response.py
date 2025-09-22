import pychrono as chrono
import pychrono.ros as chros
import numpy as np

def main():
    
    system = chrono.ChSystemNSC()
    system.SetGravity(chrono.ChVectorD(0, -9.81, 0))  
    
    
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(0.5)
    material.SetRestitution(0.2)

    
    floor = chrono.ChBody()
    floor.SetPos(chrono.ChVectorD(0, -0.5, 0))
    floor.SetBodyFixed(True)  
    floor_shape = chrono.ChBoxShape()
    floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(5, 0.5, 5)  
    floor.AddVisualShape(floor_shape)
    floor.GetCollisionModel().AddBox(material, 5, 0.5, 5)
    floor.SetCollide(True)
    system.Add(floor)

    
    box = chrono.ChBody()
    box.SetPos(chrono.ChVectorD(0, 2, 0))
    box.SetMass(1.0)
    box_shape = chrono.ChBoxShape()
    box_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.2, 0.2, 0.2)  
    box.AddVisualShape(box_shape)
    box.GetCollisionModel().AddBox(material, 0.2, 0.2, 0.2)
    box.SetCollide(True)
    system.Add(box)

    
    ros_manager = chros.ChROSPyManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())        
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(1, box))   
    ros_manager.RegisterHandler(chros.ChROSTFHandler(1, False))   

    
    class StepCounterHandler(chros.ChROSHandler):
        def __init__(self):
            super().__init__()
            self.topic_name = "/simulation/step_count"
            self.publisher = None
            self.step_count = 0
            
        def Initialize(self):
            self.publisher = ros_manager.GetNode().create_publisher(chros.Int32, self.topic_name, 1)
            return True
            
        def Tick(self, time):
            msg = chros.Int32()
            msg.data = self.step_count
            self.publisher.publish(msg)
            self.step_count += 1

    ros_manager.RegisterHandler(StepCounterHandler())

    
    if not ros_manager.Initialize():
        print("ROS initialization failed!")
        return

    
    time_step = 0.01
    realtime_step = True  

    while ros_manager.NodeOkay():
        
        system.DoStepDynamics(time_step)
        
        
        ros_manager.Update(system, system.GetChTime())

        
        if realtime_step:
            ros_manager.SpinSome()
            ros_manager.Rate().sleep()

if __name__ == "__main__":
    main()