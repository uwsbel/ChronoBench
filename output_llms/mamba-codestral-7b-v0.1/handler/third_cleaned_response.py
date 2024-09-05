import pychrono as ch
import pychrono.irrlicht as chirr

import rclpy.publisher
from std_msgs.msg import Int64


class MyCustomHandler(chirr.ChIrrlichtHandler):
    

    def __init__(self, topic, publish_rate):
        super().__init__(1)  

        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.ticker = 0  
        self.publish_rate = publish_rate

    def Initialize(self, interface: chirr.ChIrrlichtInterface) -> bool:
        
        print(f"Creating publisher for topic {self.topic} ...")
        
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 1)
        return True  

    def Tick(self, time: float):
        
        if self.ticker % self.publish_rate == 0:
            print(f"Publishing {self.ticker} ...")
            msg = Int64()  
            msg.data = self.ticker  
            self.publisher.publish(msg)  
        self.ticker += 1  

def main():
    
    sys = ch.ChSystemNSC()
    sys.SetGravitationalAcceleration(ch.ChVector3D(0, 0, -9.81))  

    
    phys_mat = ch.ChContactMaterialNSC()
    phys_mat.SetFriction(0.5)  

    
    floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
    floor.SetPos(ch.ChVector3D(0, 0, -1))  
    floor.SetFixed(True)  
    floor.SetName("base_link")  
    sys.Add(floor)  

    
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3D(0, 0, 5))  
    box.SetRot(ch.Q_from_AngAxis(.2, ch.ChVector3D(1, 0, 0)))  
    box.SetName("box")  
    sys.Add(box)  

    
    floor_texture = ch.ChTexture()
    floor_texture.SetTextureFilename("path_to_floor_texture.png")
    floor.AddAsset(floor_texture)

    box_texture = ch.ChTexture()
    box_texture.SetTextureFilename("path_to_box_texture.png")
    box.AddAsset(box_texture)

    
    ros_manager = chirr.ChIrrlichtManager()

    
    ros_manager.RegisterHandler(chirr.ChIrrlichtClockHandler())

    
    ros_manager.RegisterHandler(chirr.ChIrrlichtBodyHandler(25, box, "~/box"))

    
    tf_handler = chirr.ChIrrlichtTFHandler(30)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)

    
    custom_handler = MyCustomHandler("~/my_topic", 10)  
    ros_manager.RegisterPythonHandler(custom_handler)

    
    ros_manager.Initialize()

    
    app = chirr.ChIrrlichtApplication()
    app.SetWindowSize(800, 600)  
    app.SetWindowCaption("PyChrono Irrlicht Example")  
    app.AddLogo(chirr.ChIrrlichtLogo())  
    app.AddSkyBox()  
    app.AddTypicalLights()  
    app.AddCamera(chirr.ChIrrlichtCamera(ch.ChVector3D(0, 0, 10), ch.ChVector3D(0, 0, 0)))  

    
    time = 0
    time_step = 1e-3  
    time_end = 30  

    realtime_timer = ch.ChRealtimeStepTimer()  
    step_number = 0
    render_step_size = 10  
    render_steps = 0
    while time < time_end:
        sys.DoStepDynamics(time_step)  
        time = sys.GetChTime()  

        if not ros_manager.Update(time, time_step):  
            break  

        realtime_timer.Spin(time_step)  

        
        if render_steps >= render_step_size:
            app.BeginScene()
            app.Render()
            app.EndScene()
            render_steps = 0
        else:
            render_steps += 1

        step_number += 1


if __name__ == "__main__":
    main()