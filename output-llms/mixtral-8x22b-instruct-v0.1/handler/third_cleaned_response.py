import pychrono as ch
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros

import rclpy.publisher
from std_msgs.msg import Int64


class MyCustomHandler(chros.ChROSHandler):
    

    def __init__(self, topic, publish_rate):
        super().__init__(publish_rate)  

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
    floor.SetTexture(chronoirr.GetChronoDataFile("textures/concrete.jpg"))  
    sys.Add(floor)  

    
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 5))  
    box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))  
    box.SetName("box")  
    box.SetTexture(chronoirr.GetChronoDataFile("textures/brick.jpg"))  
    sys.Add(box)  

    
    ros_manager = chros.ChROSPythonManager()

    
    ros_manager.RegisterHandler(chros.ChROSClockHandler())

    
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/box"))

    
    tf_handler = chros.ChROSTFHandler(30)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)

    
    custom_handler = MyCustomHandler("~/my_topic", 10)  
    ros_manager.RegisterPythonHandler(custom_handler)

    
    ros_manager.Initialize()

    
    vis = chronoirr.ChIrrApp(sys, "PyChrono ROS Example", chronoirr.dimension2du(800, 600))
    vis.AddTypicalSky()
    vis.AddTypicalLogo()
    vis.AddTypicalCamera(ch.ChVector3d(0, 0, 5))
    vis.AddLightWithShadow(ch.ChVector3d(0, 0, 5), 0.5, 10, 50, 20, 50, 200, ch.ChColor(0.8, 0.8, 0.8))
    vis.AddLightWithShadow(ch.ChVector3d(-5, 5, 5), 0.5, 10, 50, 20, 50, 200, ch.ChColor(0.8, 0.8, 0.8))

    
    step_number = 0
    render_step_size = 5
    render_steps = 10

    
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
            vis.BeginScene()
            vis.DrawAll()
            vis.EndScene()

        step_number += 1
        realtime_timer.Spin(time_step)  


if __name__ == "__main__":
    main()