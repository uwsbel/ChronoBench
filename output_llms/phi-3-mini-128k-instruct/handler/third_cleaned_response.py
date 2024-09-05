import pychrono as ch
import pychrono.ros as chros
import rclpy.node
from std_msgs.msg import Int64


class MyCustomHandler(chros.ChROSHandler):
    

    def __init__(self, topic, publish_rate):
        super().__init__(publish_rate)  

        self.topic = topic
        self.publisher: rclpy.node.Publisher = None
        self.ticker = 0  

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

def main():
    
    sys = ch.ChSystemNSC()
    sys.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.81))  

    
    phys_mat = ch.ChContactMaterialNSC()
    phys_mat.SetFriction(0.5)  

    
    floor_texture = ch.ChTexture("path/to/floor_texture.jpg")
    floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
    floor.SetPos(ch.ChVector3d(0, 0, -1))  
    floor.SetTexture(floor_texture)  
    floor.SetFixed(True)  
    floor.SetName("base_link")  
    sys.Add(floor)  

    
    box_texture = ch.ChTexture("path/to/box_texture.jpg")
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 5))  
    box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))  
    box.SetTexture(box_texture)  
    box.SetName("box")  
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

    
    irr_scene = ch.IrrlichtSceneManager()
    irr_scene.AddUniverse(ch.IrrlichtDevice::SINGLE_EXAMPLE)
    irr_scene.AddCamera("main_camera", ch.IrrlichtDevice::UNIVERSE_FOREGROUND)
    irr_scene.AddLight("main_light", ch.IrrlichtDevice::UNIVERSE_FOREGROUND)
    irr_scene.AddWindow("window", 1024, 768, "PyChrono Visualization", ch.IrrlichtDevice::WINDOW_EXCLUDE_MENU)

    
    irr_scene.SetRenderWindow(ch.IrrlichtDevice::RenderWnd_EXCLUDE_MENU)
    irr_scene.SetWindowShown(True)
    irr_scene.AddCameraSceneNode(ch.IrrlichtDevice::UNIVERSE_FOREGROUND)

    
    floor_mesh = irr_scene.AddMesh("path/to/floor_mesh.obj", ch.IrrlichtDevice::UNIVERSE_FOREGROUND)
    box_mesh = irr_scene.AddMesh("path/to/box_mesh.obj", ch.IrrlichtDevice::UNIVERSE_FOREGROUND)
    floor_mesh.SetTexture("path/to/floor_texture.jpg")
    box_mesh.SetTexture("path/to/box_texture.jpg")

    
    step_number = 0
    render_step_size = 10
    render_steps = 100

    
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
            irr_scene.RenderWindow()
            step_number += 1

        realtime_timer.Spin(time_step)  


if __name__ == "__main__":
    main()