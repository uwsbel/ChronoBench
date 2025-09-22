import math
import pychrono as ch
from   pychrono import irrlicht as irr                      
import pychrono.ros as chros

import rclpy
import rclpy.publisher
from   std_msgs.msg import Int64






def add_texture(body, filepath):
    
    texture = ch.ChTexture()
    texture.SetTextureFilename(filepath)
    body.AddAsset(texture)






publish_rate = 10     

class MyCustomHandler(chros.ChROSHandler):
    

    def __init__(self, topic: str):
        super().__init__(publish_rate)                     
        self.topic     = topic
        self.publisher : rclpy.publisher.Publisher = None
        self.ticker    = 0

    
    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        print(f"[ROS] Creating publisher on {self.topic}")
        node = interface.GetNode()
        self.publisher = node.create_publisher(Int64, self.topic, 10)
        return True

    
    def Tick(self, time: float):
        msg       = Int64()
        msg.data  = self.ticker
        self.publisher.publish(msg)
        print(f"[ROS] Published {self.ticker} on {self.topic}")
        self.ticker += 1






def main():

    
    sys = ch.ChSystemNSC()
    sys.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.81))

    
    mat = ch.ChContactMaterialNSC()
    mat.SetFriction(0.5)

    
    
    floor = ch.ChBodyEasyBox(10, 10, 1,            
                             1000,                 
                             True, True, mat)
    floor.SetPos(ch.ChVector3d(0, 0, -1))
    floor.SetFixed(True)
    floor.SetName("base_link")
    add_texture(floor, ch.GetChronoDataFile("textures/concrete.jpg"))
    sys.Add(floor)

    
    box = ch.ChBodyEasyBox(1, 1, 1,
                           1000,
                           True, True, mat)
    box.SetPos(ch.ChVector3d(0, 0, 5))
    
    box.SetRot(ch.Q_from_AngAxis(0.2, ch.ChVector3d(1, 0, 0)))
    box.SetName("box")
    add_texture(box, ch.GetChronoDataFile("textures/cubetexture_bluewhite.png"))
    sys.Add(box)

    
    ros_manager = chros.ChROSPythonManager()

    
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(publish_rate, box, "~/box"))

    tf_handler = chros.ChROSTFHandler(publish_rate)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)

    
    custom_handler = MyCustomHandler("~/my_topic")
    ros_manager.RegisterPythonHandler(custom_handler)

    ros_manager.Initialize()

    
    app = irr.ChIrrApp(sys,
                       "PyChrono — ROS/Irrlicht demo",
                       irr.dimension2du(1024, 768))

    app.AddTypicalSky()
    app.AddTypicalLights()
    app.AddTypicalCamera(irr.vector3df(3, 3, 3),
                         irr.vector3df(0, 0, 0))

    app.AssetBindAll()
    app.AssetUpdateAll()

    
    time_step   = 1e-3                                 
    time_end    = 30.0

    
    step_number       = 0                              
    render_step_size  = 1.0 / 60.0                     
    render_steps      = int(render_step_size / time_step)

    realtime_timer = ch.ChRealtimeStepTimer()

    while sys.GetChTime() < time_end:

        
        if not app.GetDevice().run():
            break

        
        sys.DoStepDynamics(time_step)

        
        if not ros_manager.Update(sys.GetChTime(), time_step):
            break

        
        if step_number % render_steps == 0:
            app.BeginScene()
            app.DrawAll()
            app.EndScene()

        step_number += 1
        realtime_timer.Spin(time_step)                




if __name__ == "__main__":
    main()