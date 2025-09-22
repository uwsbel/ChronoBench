import pychrono as ch
import pychrono.ros as chros
import pyrr
import Irrlicht

import rclpy.publisher
from std_msgs.msg import Int64


class MyCustomHandler(chros.ChROSHandler):
    

    def __init__(self, topic):
        super().__init__(10)  

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

    
    floor_geometry = ch.ChBoxShape()
    floor_geometry.SetSize(ch.ChVectorD(10, 10, 0.1))
    floor_material = ch.ChMaterialSurfaceNSC()
    floor_material.SetFriction(0.5)
    floor_body = ch.ChBodyEasyBox(10, 10, 0.1, 1000, True, True, phys_mat, floor_material)
    floor_body.SetPos(ch.ChVector3d(0, 0, -1))  
    floor_body.SetFixed(True)  
    floor_body.SetName("base_link")  
    floor_body.SetTexture(ch.ChTexture("path/to/floor_texture.png"))
    sys.Add(floor_body)  

    
    box_geometry = ch.ChBoxShape()
    box_geometry.SetSize(ch.ChVectorD(1, 1, 1))
    box_material = ch.ChMaterialSurfaceNSC()
    box_material.SetFriction(0.5)
    box_body = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat, box_material)
    box_body.SetPos(ch.ChVector3d(0, 0, 5))  
    box_body.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))  
    box_body.SetName("box")  
    box_body.SetTexture(ch.ChTexture("path/to/box_texture.png"))
    sys.Add(box_body)  

    
    publish_rate = 10  
    ros_manager = chros.ChROSPythonManager()

    
    ros_manager.RegisterHandler(chros.ChROSClockHandler())

    
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(publish_rate, box_body, "~/box"))

    
    tf_handler = chros.ChROSTFHandler(publish_rate)
    tf_handler.AddTransform(floor_body, floor_body.GetName(), box_body, box_body.GetName())
    ros_manager.RegisterHandler(tf_handler)

    
    custom_handler = MyCustomHandler("~/my_topic")
    ros_manager.RegisterPythonHandler(custom_handler)

    
    ros_manager.Initialize()

    
    device = Irrlicht.createDevice(
        Irrlicht.video.EDT_OPENGL,
        core.dim2d(800, 600),
        16,
        False,
        False,
        False,
        Irrlicht.scene.E_SCENE_MANAGER_AUTOCLEAR_FLAG | Irrlicht.video.E_VIDEO_RENDERER_PRESENT_RATES,
    )

    driver = device.getVideoDriver()
    smgr = device.getSceneManager()

    camera = smgr.addCameraSceneNodeFPS()
    camera.setPosition(pyrr.Vector3([0, 5, -10]))
    camera.setTarget(pyrr.Vector3([0, 0, 0]))

    light_ambient = smgr.addLightSceneNode(None, pyrr.Vector3([0, 0, 0]))
    light_ambient.setLightType(Irrlicht.video.ELT_DIRECTIONAL)
    light_ambient.setColor(Irrlicht.video.SColor(255, 255, 255, 255))

    light_point = smgr.addLightSceneNode(None, pyrr.Vector3([0, 5, 0]))
    light_point.setLightType(Irrlicht.video.ELT_POINT)
    light_point.setColor(Irrlicht.video.SColor(255, 255, 255, 255))

    
    step_number = 0
    render_step_size = 5
    render_steps = 1

    
    time = 0
    time_step = 1e-3  
    time_end = 30  

    realtime_timer = ch.ChRealtimeStepTimer()  
    while time < time_end:
        sys.DoStepDynamics(time_step)  
        time = sys.GetChTime()  

        if step_number % render_step_size == 0:
            
            driver.beginScene(True, True, Irrlicht.video.SColor(255, 100, 100, 100))

            smgr.drawAll()
            device.setWindowCaption(f"Chrono Simulation - Time: {time:.2f}")

            driver.endScene()

        if not ros_manager.Update(time, time_step):  
            break  

        realtime_timer.Spin(time_step)  

        step_number += 1


if __name__ == "__main__":
    main()