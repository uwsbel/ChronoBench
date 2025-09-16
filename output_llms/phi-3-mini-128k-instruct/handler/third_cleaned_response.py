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
from irrlicht.scene.render import RenderWindow, RenderScene, RenderPlugin
from irrlicht.scene.scene import Scene
from irrlicht.scene.video import VideoDriver
from irrlicht.scene.video.video_device import VideoDeviceType
from irrlicht.scene.video.video_device import VideoDriver
from irrlicht.scene.video.video_device import VideoDriverType
from irrlicht.scene.video.video_device import VideoDriverType_2D


class MyCustomHandler(chros.ChROSHandler):
    

    def __init__(self, topic, publish_rate):
        super().__init__(publish_rate)  

        self.topic = topic
        self.publisher: Publisher = None
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

    
    floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
    floor.SetPos(ch.ChVector3d(0, 0, -1))  
    floor.SetTexture("path_to_floor_texture.jpg")  
    floor.SetName("base_link")  
    sys.Add(floor)  

    
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 5))  
    box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))  
    box.SetTexture("path_to_box_texture.jpg")  
    box.SetName("box")  
    sys.Add(box)  

    
    scene = Scene()
    video_device = VideoDriver(VideoDriverType.DIRECT3D, VideoDriverType_2D)
    video_driver = VideoDriver(scene, video_device)
    video_driver.SetFullscreen(False)
    video_driver.SetWidth(1280)
    video_driver.SetHeight(720)
    video_driver.SetFPS(60)
    video_driver.SetAntiAlias(True)
    video_driver.SetDeviceName("Direct3D")
    video_driver.SetFullscreen(False)
    video_driver.SetBackgroundColor(ch.vector3df(0.1, 0.1, 0.1))

    
    camera = CameraInfo()
    camera.SetPosition(ch.vector3df(5, 5, 5))
    camera.SetFov(ch.vector3df(60, 45))
    camera.SetTarget(ch.vector3df(0, 0, 0))
    camera.SetFOV(ch.vector3df(90, 60))
    camera.SetFieldOfView(ch.vector3df(30, 0, 0))
    camera.SetNearPlane(0.1)
    camera.SetFarPlane(1000)
    scene.AddLight(LightInfo(LightType.DIRECTIONAL, ch.vector3df(1, 1, 1), 1.0)
    scene.AddLight(LightInfo(LightType.DIRECTIONAL, ch.vector3df(-1, -1, -1), 0.5))
    scene.AddLight(LightInfo(LightType.DIRECTIONAL, ch.vector3df(1, -1, -1), 0.5))
    scene.AddLight(LightInfo(LightType.DIRECTIONAL, ch.vector3df(-1, 1, -1), 0.5))
    scene.AddCamera(camera)

    
    render_scene = RenderScene()
    render_scene.AddGeometry(floor)
    render_scene.AddGeometry(box)
    render_scene.SetSkySea()
    render_scene.SetBackgroundColor(ch.vector3df(0.1, 0.1, 0.1))
    render_scene.SetAmbientLight(ch.vector3df(0.3, 0.3, 0.3))
    render_scene.SetLight(ch.vector3df(1, 1, 1))

    
    render_window = RenderWindow(video_driver, "PyChrono Visualization", 1280, 720)
    render_window.AddLight(LightInfo(LightType.DIRECTIONAL, ch.vector3df(1, 1, 1), 1.0))
    render_window.AddCamera(camera)

    
    step_number = 0
    render_steps = 5
    step_time = 1.0 / render_steps
    render_timer = Timer(get_clock(), step_time)

    
    ros_manager = chros.ChROSPythonManager()

    
    ros_manager.RegisterHandler(chros.ChROSClockHandler())

    
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/box"))

    
    tf_handler = chros.ChROSTFHandler(30)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)

    
    custom_handler = MyCustomHandler("~/my_topic", 10)  
    ros_manager.RegisterPythonHandler(custom_handler)

    
    ros_manager.Initialize()

    
    node = Node("pychrono_node")
    publisher = Publisher(node, "/my_topic", Int64(), 10)  

    
    time = 0
    time_step = 1e-3  
    time_end = 30  

    while time < time_end:
        sys.DoStepDynamics(time_step)  
        time = sys.GetChTime()  

        
        ros_manager.Update(time, time_step)

        
        if step_number % render_steps == 0:
            render_scene.RenderScene()
            render_window.RenderScene()
            step_number += 1

        realtime_timer.Spin(time_step)  

if __name__ == "__main__":
    main()