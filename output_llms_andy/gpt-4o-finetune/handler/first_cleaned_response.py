import pychrono as chrono
import pychrono.robot as chrrobot
import pychrono.ros as chros
import numpy as np

def main():
    
    
    
    
    sys = chrono.ChSystemNSC()
    mat = chrono.ChContactMaterialNSC()
    mat.SetFriction(0.5)  
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.GetSettings().collision.collision_envelope = 0.01  
    sys.GetSettings().collision.narrowphase_algorithm = (
        chrono.ChNarrowPhaseCollider.Algorithm_MANIFOLD  
    )
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  

    
    
    
    
    
    
    floor_body = chrono.ChBodyEasyBox(
        20, 20, 1,  
        1000,       
        True,       
        True,       
        mat         
    )
    floor_body.SetPos(chrono.ChVector3d(0, 0, -1))  
    floor_body.SetFixed(True)  
    floor_body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  
    sys.Add(floor_body)  

    
    
    
    
    
    
    box = chrono.ChBodyEasyBox(
        1, 1, 1,  
        1000,     
        True,     
        True,     
        mat       
    )
    box.SetPos(chrono.ChVector3d(0, 0, 2))  
    box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))  
    sys.Add(box)  

    
    
    
    
    ros_manager = chros.ChROSPythonManager()

    
    
    
    class MyCustomHandler(chros.ChROS2CustomHandler):
        

        def __init__(self, topic_name):
            super().__init__(
                1
            )  
            self.topic_name = topic_name
            self.publisher = None  
            self.msg = chrono.ChInt32()  
            self.count = 0  

        def OnInitialize(self):
            
            
            self.publisher = self.GetNode().CreatePublisher(
                self.topic_name, chrono.ChInt32.GetMaxStampedMessageType()
            )

        def OnUpdate(self, time):
            
            self.msg.data = self.count  
            self.publisher.Publish(self.msg)  
            print(f"Publishing: {self.count}")  
            self.count += 1  

    
    
    custom_handler = MyCustomHandler("my_topic")
    ros_manager.RegisterHandler(chros.ChROSClockHandler())  
    ros_manager.RegisterHandler(
        chros.ChROSBodyHandler(sys.GetChTime())
    )  
    ros_manager.RegisterHandler(
        chros.ChROSTFHandler(sys.GetChTime())
    )  
    ros_manager.RegisterHandler(
        custom_handler
    )  
    ros_manager.Initialize()  

    
    
    
    
    
    time_step = 1e-3  
    time = 0  
    time_out = 0  
    time_acc = 0  

    
    
    
    
    
    max_time = 20  
    time_end = sys.GetChTime() + max_time  
    steps = int((time_end - sys.GetChTime()) / time_step)  

    for i in range(steps):
        sys.DoStepDynamics(time_step)  
        if not ros_manager.Update():
            break  
        time = sys.GetChTime() - time_out  
        time_acc += time  
        
        time_sleep = max(0, (i + 1) * time_step - time_acc)
        chrono.ChTime().Sleep(time_sleep)  

if __name__ == "__main__":
    main()