import pychrono as ch
import pychrono.ros as chros
import pychrono irrlicht as ch irr  

import rclpy.publisher
from std_msgs.msg import Int64


class MyCustomHandler(chros.ChROSHandler):
    

    def __init__(self, topic):
        super().__init__(1)  

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

    
    vis = ch irr.ChIrrApp(sys, "Simulation Window", ch irr.VEC2(800, 600))  
    vis.SetCameraPosition(ch irr.Pt3d(0, 5, 10))  
    vis.SetCameraLookAt(ch irr.Pt3d(0, 0, 0))  
    vis.AddLight(ch irr.Pt3d(0, 5, 10), ch irr.Pt3d(0, 0, 0), 100)  
    vis.AddTypicalLogo()  
    vis.AddTypicalSky()  
    vis.AddTypicalGrid()  
    vis.SetFrameRate(60)  

    
    phys_mat = ch.ChContactMaterialNSC()
    phys_mat.SetFriction(0.5)  

    
    floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
    floor.SetPos(ch.ChVector3d(0, 0, -1))  
    floor.SetFixed(True)  
    floor.SetName("base_link")  
    floor.SetTexture(ch.Texture("textures/stone.jpg"))  
    sys.Add(floor)  

    
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 5))  
    box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))  
    box.SetName("box")  
    box.SetTexture(ch.Texture("textures/metal.jpg"))  
    sys.Add(box)  

    
    ros_manager = chros.ChROSPythonManager()
    
    
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    
    
    publish_rate = 10  
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(publish_rate, box, "~/box"))
    
    
    tf_handler = chros.ChROSTFHandler(publish_rate)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)
    
    
    custom_handler = MyCustomHandler("~/my_topic")
    custom_handler.SetRate(publish_rate)  
    ros_manager.RegisterPythonHandler(custom_handler)

    
    ros_manager.Initialize()

    
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

        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        step_number += 1
        render_steps += 1
        if render_steps >= render_step_size:
            vis.Render()
            render_steps = 0

        
        vis.HandleEvents()


if __name__ == "__main__":
    main()