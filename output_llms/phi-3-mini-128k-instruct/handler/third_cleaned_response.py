import pychrono as ch
import pychrono.ros as chros
import rclpy.publisher
from std_msgs.msg import Int64
from rclpy.node import Node
from rclpy.node import Publisher
from rclpy.node import Subscription
from rclpy.node import Node
from rclpy.node import get_clock
from rclpy.timer import Timer
from rclpy.time import Time
from irrlicht.scene.mesh import Mesh
from irrlicht.scene.render import RenderWindow, RenderScene, RenderDevice, RenderType
from irrlicht.scene.material import Material
from irrlicht.scene.light import DirectionalLight
from irrlicht.scene.video import VideoDriver
from irrlicht.scene.video import VideoDriverType


class MyCustomHandler(chros.ChROSHandler):
    

    def __init__(self, topic: str, publish_rate: float):
        super().__init__(publish_rate)  

        self.topic = topic
        self.publisher: Publisher = None
        self.ticker = 0  

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        
        print(f"Creating publisher for topic {self.topic} ...")
        
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, self.publish_rate)
        return True  

    def Tick(self, time: float) -> None:
        
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
    floor.SetTexture("path/to/floor_texture.jpg")  
    floor.SetName("base_link")  
    sys.Add(floor)  

    
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 5))  
    box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))  
    box.SetTexture("path/to/box_texture.jpg")  
    box.SetName("box")  
    sys.Add(box)  

    
    ros_manager = chros.ChROSPythonManager()
    
    
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    
    
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/box"))
    
    
    irrlicht_scene = Mesh.addFilledMesh(Mesh.createPlane(8, 8, 1))
    irrlicht_scene.SetMaterial(Material.New(0, Material.CreateMipMap(Material.New(0, 0.5, 0.5, 1, 1, 1)))
    irrlicht_scene.SetTexture("path/to/floor_texture.jpg")
    irrlicht_scene.SetTexture("path/to/box_texture.jpg")

    
    render_device = RenderDevice(VideoDriver(VideoDriverType.DIRECTVIDEO))
    render_window = RenderWindow(800, 600, "PyChrono Simulation", "DirectWindow", render_device)
    render_scene = RenderScene(render_window, RenderType.GREENSCREEN)
    render_scene.AddLight(DirectionalLight(LightType.DIRECTIONAL, DirectionalLight.CreateBlue(255, 255, 255, 100))
    render_scene.AddCamera("irrlicht_camera", 50, 50, 50)

    
    step_number = 0
    render_step_size = 100
    render_steps = 10
    timer = Timer(get_clock(), render_step_size, True)

    
    ros_manager.Initialize()

    
    time = 0
    time_step = 1e-3  
    time_end = 30  

    realtime_timer = ch.ChRealtimeStepTimer()  
    while time < time_end:
        sys.DoStepDynamics(time_step)  
        time = sys.GetChTime()  

        if not ros_manager.Update(time, time_step):  
            break  

        
        if step_number % render_steps == 0:
            render_scene.RenderScene()
            timer.reset()

        
        ros_manager.Update(time, time_step)

        
        irrlicht_scene.SetCamera("irrlicht_camera")
        irrlicht_scene.SetLightPosition(ch.ChVector3d(10, 10, 10))

        step_number += 1

    
    timer.cancel()
    render_scene.DeleteAllActors()
    render_window.Close()

if __name__ == "__main__":
    rclpy.init(args=None)
    node = Node("pychrono_ros_visualization")
    custom_handler = MyCustomHandler("~/my_topic", 10)  
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterPythonHandler(custom_handler)
    ros_manager.Initialize()
    main()