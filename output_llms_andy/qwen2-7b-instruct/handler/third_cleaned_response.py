import pychrono as ch
import pychrono.ros as chros

import rclpy
from std_msgs.msg import Int64


class MyCustomHandler(chros.ChROSHandler):
    

    def __init__(self, topic):
        super().__init__(10)  

        self.topic = topic
        self.publisher = None
        self.ticker = 0  

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        
        print(f"Creating publisher for topic {self.topic} ...")
        
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 1)
        return True  

    def Tick(self, time: float):
        
        print(f"Publishing {self.ticker} ...")
        msg = Int64()  
        msg.data = self.ticker  
        self.publisher.publish(msg)  
        self.ticker += 1  

def main():
    
    sys = ch.ChSystemNSC()
    sys.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.81))  

    
    phys_mat = ch.ChContactMaterialNSC()
    phys_mat.SetFriction(0.5)  

    
    floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
    floor.SetPos(ch.ChVector3d(0, 0, -1))  
    floor.SetFixed(True)  
    floor.SetName("base_link")  
    sys.Add(floor)  

    
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 5))  
    box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))  
    box.SetName("box")  
    sys.Add(box)  

    
    ros_manager = chros.ChROSPythonManager()

    
    ros_manager.RegisterHandler(chros.ChROSClockHandler())

    
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/box"))

    
    tf_handler = chros.ChROSTFHandler(30)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)

    
    custom_handler = MyCustomHandler("~/my_topic")
    ros_manager.RegisterPythonHandler(custom_handler)

    
    ros_manager.Initialize()

    
    sys.SetVisualSystem(ch.ChVisualSystemIrrlicht())
    sys.GetVisualSystem().SetWindowFlags(ch.IrrlichtWindowFlags.INVARIANT)
    sys.GetVisualSystem().SetWindowCaption("Chrono Simulation")
    sys.GetVisualSystem().SetCameraTarget(ch.ChVector3d(0, 0, 10))
    sys.GetVisualSystem().SetCameraUpVector(ch.ChVector3d(0, 1, 0))
    sys.GetVisualSystem().SetWindowSize(ch.IrrlichtWindowSize(800, 600))
    sys.GetVisualSystem().SetWindowFlags(ch.IrrlichtWindowFlags.ZBUFFER)
    sys.GetVisualSystem().AddLight(ch.ChVector3d(10, 10, 10), ch.ChVector3d(1, 1, 1))
    sys.GetVisualSystem().AddLight(ch.ChVector3d(-10, -10, -10), ch.ChVector3d(1, 1, 1))
    sys.GetVisualSystem().AddTexture("floor_texture", "path/to/floor_texture.png")
    sys.GetVisualSystem().AddTexture("box_texture", "path/to/box_texture.png")
    sys.GetVisualSystem().SetTexture(floor, "floor_texture")
    sys.GetVisualSystem().SetTexture(box, "box_texture")

    
    step_number = 0
    render_step_size = 10
    render_steps = 30

    
    time = 0
    time_step = 1e-3  
    time_end = 30  

    realtime_timer = ch.ChRealtimeStepTimer()  
    while time < time_end:
        sys.DoStepDynamics(time_step)  
        time = sys.GetChTime()  

        if not ros_manager.Update(time, time_step):  
            break  

        if step_number % render_step_size == 0:
            sys.GetVisualSystem().Render()
            step_number += 1

        realtime_timer.Spin(time_step)  


if __name__ == "__main__":
    main()