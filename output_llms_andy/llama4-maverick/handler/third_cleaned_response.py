import pychrono as ch
import pychrono.ros as chros
import pychrono.irrlicht as chirr

import rclpy
from std_msgs.msg import Int64


rclpy.init()


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

    
    floor_asset = floor.GetAssets()[0]
    floor_visual_material = ch.ChVisualMaterial()
    floor_visual_material.SetKdTexture("path_to_floor_texture.jpg")  
    floor_asset.AddMaterial(floor_visual_material)

    sys.Add(floor)  

    
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 5))  
    box.SetRot(ch.QuatFromAngleAxis(0.2, ch.ChVector3d(1, 0, 0)))  
    box.SetName("box")  

    
    box_asset = box.GetAssets()[0]
    box_visual_material = ch.ChVisualMaterial()
    box_visual_material.SetKdTexture("path_to_box_texture.jpg")  
    box_asset.AddMaterial(box_visual_material)

    sys.Add(box)  

    
    ros_manager = chros.ChROSPythonManager()

    
    ros_manager.RegisterHandler(chros.ChROSClockHandler())

    
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/box"))

    
    tf_handler = chros.ChROSTFHandler(30)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)

    
    publish_rate = 10

    
    custom_handler = MyCustomHandler("~/my_topic", publish_rate)
    ros_manager.RegisterPythonHandler(custom_handler)

    
    ros_manager.Initialize()

    
    vis = chirr.ChVisualSystemIrrlicht()
    vis.SetWindowSize(800, 600)
    vis.SetWindowTitle('Chrono Simulation')
    vis.Initialize()
    vis.AddLogo()
    vis.AddSkyBox()
    vis.AddCamera(ch.ChVector3d(0, -5, 3), ch.ChVector3d(0, 0, 0))
    vis.AddTypicalLights()
    vis.AttachSystem(sys)

    
    step_number = 0
    render_step_size = 0.01
    render_steps = int(render_step_size / 1e-3)

    
    time = 0
    time_step = 1e-3  
    time_end = 30  

    realtime_timer = ch.ChRealtimeStepTimer()  
    while time < time_end:
        sys.DoStepDynamics(time_step)  
        time = sys.GetChTime()  

        if not ros_manager.Update(time, time_step):  
            break  

        step_number += 1
        if step_number % render_steps == 0:
            vis.Run()  
            vis.Render()

        realtime_timer.Spin(time_step)  

    
    rclpy.shutdown()


if __name__ == "__main__":
    main()