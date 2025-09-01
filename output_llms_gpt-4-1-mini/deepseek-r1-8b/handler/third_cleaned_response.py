import pychrono as ch
import pychrono.ros as chros
import rclpy.publisher
from std_msgs.msg import Int64
import chvis
from pychrono.ros.chros_handler import ChROSHandler
import time


class MyCustomHandler(ChROSHandler):
    

    def __init__(self, topic):
        super().__init__(1)  
        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.ticker = 0  
        self.publish_rate = 10  

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        
        print(f"Creating publisher for topic {self.topic} ...")
        
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, self.publish_rate)
        return True  

    def Tick(self, time: float):
        
        print(f"Publishing {self.ticker} ...")
        msg = Int64()  
        msg.data = self.ticker  
        self.publisher.publish(msg)  
        self.ticker += 1  
        
        if self.ticker >= 10:
            self.ticker = 0

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

    
    scene = chvis.Scene()
    scene.background = chvis.Color(0, 0, 0)
    camera = scene.add_camera("perspective", position=(5, 5, 5), look_at=(0, 0, 0))
    light = scene.add_light("directional", position=(10, 10, 10), color=(1, 1, 1))
    window = scene.add_window()
    window.set_size(800, 600)
    window.set_position(chvis.Vector2i(100, 100))
    window.set_caption("PyChrono ROS Simulation")

    
    floor_texture_path = "floor_texture.png"
    box_texture_path = "box_texture.png"
    floor.SetTexture(chvis.Texture2D(floor_texture_path, True))
    box.SetTexture(chvis.Texture2D(box_texture_path, True))

    
    step_number = 0
    render_step_size = 0.01  
    render_steps = 100  

    
    time = 0
    time_step = 1e-3  
    time_end = 30  

    realtime_timer = ch.ChRealtimeStepTimer()  
    while time < time_end:
        sys.DoStepDynamics(time_step)  
        time = sys.GetChTime()  

        
        if step_number % render_steps == 0:
            
            scene.update()
            step_number += 1

        if not ros_manager.Update(time, time_step):  
            break  

        realtime_timer.Spin(time_step)  


if __name__ == "__main__":
    main()