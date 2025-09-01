import pychrono as ch
import pychrono.ros as chros

import rclpy.publisher
from std_msgs.msg import Int64


class MyCustomHandler(chros.ChROSHandler):
    

    def __init__(self, topic, publish_rate=10):
        super().__init__(publish_rate)  

        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.ticker = 0  

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        
        print(f"Creating publisher for topic {self.topic} ...")
        
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 10)
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
    
    floor_texture_path = "path/to/floor_texture.jpg"  
    floor.GetAssets().push_back(ch.ChTexture().SetFilename(floor_texture_path))
    sys.Add(floor)  

    
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 5))  
    box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))  
    box.SetName("box")  
    
    box_texture_path = "path/to/box_texture.jpg"  
    box.GetAssets().push_back(ch.ChTexture().SetFilename(box_texture_path))
    sys.Add(box)  

    
    vis = ch.VisualizationIrrlicht()
    vis.GetDevice().has_keyboard = True
    vis.GetDevice().open_window(title='PyChrono Simulation', width=800, height=600, window_pos_x=100, window_pos_y=100)
    vis.Initialize()

    
    camera = vis.GetDevice().recipient()
    camera.set_camera(look_at=ch.ChVector3d(0, -10, 5), position=ch.ChVector3d(0, -10, 5))
    
    
    light = ch.ChLighting()
    light.set_light_direction(ch.ChVector3d(0, -10, 10))
    vis.GetDevice().add_lights([light])

    
    ros_manager = chros.ChROSPythonManager()
    
    
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    
    
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/box"))
    
    
    tf_handler = chros.ChROSTFHandler(30)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)
    
    
    publish_rate = 10  
    custom_handler = MyCustomHandler("~/my_topic", publish_rate=publish_rate)
    ros_manager.RegisterPythonHandler(custom_handler)

    
    ros_manager.Initialize()

    
    time = 0
    time_step = 1e-3  
    time_end = 30

    
    step_number = 0
    render_step_size = 10  
    render_steps = 0  

    realtime_timer = ch.ChRealtimeStepTimer()  
    while time < time_end:
        sys.DoStepDynamics(time_step)  
        time = sys.GetChTime()  

        
        if not ros_manager.Update(time, time_step):
            break

        
        if step_number % render_step_size == 0:
            
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            render_steps += 1

        
        step_number += 1

        
        realtime_timer.Spin(time_step)

    
    vis.GetDevice().close()

if __name__ == "__main__":
    main()