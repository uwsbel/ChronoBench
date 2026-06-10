import pychrono as ch
import pychrono.ros as chros
import pychrono.irrlicht as chirr

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
        return True  

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

    
    floor_asset = floor.GetVisualShape(0)
    if floor_asset:
        floor_asset.SetTexture(ch.GetChronoDataFile("textures/concrete.jpg"))

    
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 5))  
    box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))  
    box.SetName("box")  
    sys.Add(box)  

    
    box_asset = box.GetVisualShape(0)
    if box_asset:
        box_asset.SetTexture(ch.GetChronoDataFile("textures/blue.png"))

    
    vis = chirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("PyChrono ROS Simulation")
    vis.Initialize()
    vis.AddCamera(ch.ChVector3d(0, -10, 5), ch.ChVector3d(0, 0, 0))
    vis.AddTypicalLights()
    vis.AddSkyBox()
    vis.AddLogo()

    
    publish_rate = 10  

    
    ros_manager = chros.ChROSPythonManager()

    
    ros_manager.RegisterHandler(chros.ChROSClockHandler())

    
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(publish_rate, box, "~/box"))

    
    tf_handler = chros.ChROSTFHandler(publish_rate)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)

    
    custom_handler = MyCustomHandler("~/my_topic", publish_rate)
    ros_manager.RegisterPythonHandler(custom_handler)

    
    ros_manager.Initialize()

    
    time = 0
    time_step = 1e-3  
    time_end = 30  

    
    step_number = 0
    render_frame_rate = 30  
    render_step_size = 1.0 / render_frame_rate  
    render_steps = max(1, int(render_step_size / time_step))  

    realtime_timer = ch.ChRealtimeStepTimer()  
    while time < time_end:
        
        if step_number % render_steps == 0:
            if not vis.Run():
                break
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        sys.DoStepDynamics(time_step)  
        time = sys.GetChTime()  

        if not ros_manager.Update(time, time_step):  
            break  

        realtime_timer.Spin(time_step)  
        step_number += 1  


if __name__ == "__main__":
    main()