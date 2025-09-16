import pychrono as ch
import pychrono.ros as chros
import pychrono.irrlicht as chirr

import rclpy.publisher
from std_msgs.msg import Int64


class MyCustomHandler(chros.ChROSHandler):
    

    def __init__(self, topic):
        super().__init__(10)  

        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
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
    floor.SetTexture(ch.Texture("path/to/floor_texture.png"))  
    sys.Add(floor)  

    
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 5))  
    box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))  
    box.SetName("box")  
    box.SetTexture(ch.Texture("path/to/box_texture.png"))  
    sys.Add(box)  

    
    application = chirr.ChIrrApp(sys, "PyChrono Simulation", chirr.dimension2du(1024, 768))
    application.AddTypicalLogo()
    application.AddTypicalSky()
    application.AddTypicalLights()
    application.AddTypicalCamera(chirr.vector3df(0, 0, 3))
    application.AssetBindAll()
    application.AssetUpdateAll()

    
    ros_manager = chros.ChROSPythonManager()
    publish_rate = 10  

    
    ros_manager.RegisterHandler(chros.ChROSClockHandler(publish_rate))

    
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(publish_rate, box, "~/box"))

    
    tf_handler = chros.ChROSTFHandler(publish_rate)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)

    
    custom_handler = MyCustomHandler("~/my_topic")
    ros_manager.RegisterPythonHandler(custom_handler)

    
    ros_manager.Initialize()

    
    step_number = 0
    render_step_size = 10  
    render_steps = 0

    
    time = 0
    time_step = 1e-3  
    time_end = 30  

    realtime_timer = ch.ChRealtimeStepTimer()  
    while time < time_end and application.GetDevice().run():
        sys.DoStepDynamics(time_step)  
        time = sys.GetChTime()  

        if not ros_manager.Update(time, time_step):  
            break  

        
        if step_number % render_step_size == 0:
            application.BeginScene()
            application.DrawAll()
            application.DoStep()
            application.EndScene()
            render_steps += 1

        step_number += 1
        realtime_timer.Spin(time_step)  


if __name__ == "__main__":
    main()