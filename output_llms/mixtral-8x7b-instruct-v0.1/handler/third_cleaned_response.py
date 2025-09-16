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

    
    floor_geometry = ch.ChBox(10, 10, 1)
    floor_material = ch.ChMaterialSurfaceNSC(phys_mat)
    floor_body = ch.ChBody(floor_geometry, 1000, floor_material)
    floor_body.SetPos(ch.ChVector3d(0, 0, -1))  
    floor_body.SetFixed(True)  
    floor_body.SetName("base_link")  
    floor_body.SetTexture(ch.ChTexture("path/to/floor_texture.png"))
    sys.Add(floor_body)  

    
    box_geometry = ch.ChBox(1, 1, 1)
    box_material = ch.ChMaterialSurfaceNSC(phys_mat)
    box_body = ch.ChBody(box_geometry, 1000, box_material)
    box_body.SetPos(ch.ChVector3d(0, 0, 5))  
    box_body.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))  
    box_body.SetName("box")  
    box_body.SetTexture(ch.ChTexture("path/to/box_texture.png"))
    sys.Add(box_body)  

    
    publish_rate = 10  
    ros_manager = chros.ChROSPythonManager()

    
    ros_manager.RegisterHandler(chros.ChROSClockHandler(publish_rate))

    
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(publish_rate, box_body, "~/box"))

    
    tf_handler = chros.ChROSTFHandler(publish_rate)
    tf_handler.AddTransform(floor_body, floor_body.GetName(), box_body, box_body.GetName())
    ros_manager.RegisterHandler(tf_handler)

    
    custom_handler = MyCustomHandler("~/my_topic")
    ros_manager.RegisterPythonHandler(custom_handler)

    
    device = Irrlicht.createDevice(Irrlicht.dimension2d(800, 600), 16, False, False, False, False, False)
    device.setWindowCaption("PyChrono Simulation")
    driver = device.getVideoDriver()
    scene_manager = device.getSceneManager()
    scene_manager.setAmbientLight(Irrlicht.SColor(100, 100, 100, 100))

    
    camera_node = scene_manager.addCameraSceneNode()
    camera_node.setPosition(ch.ChVectorD(5, 5, 5))
    camera_node.setTarget(ch.ChVectorD(0, 0, 0))

    
    light_node1 = scene_manager.addLightSceneNode(
        0,
        ch.ChVector3d(0, 10, 0),
        Irrlicht.SColor(255, 255, 255, 255),
        1000.0,
    )
    light_node2 = scene_manager.addLightSceneNode(
        0,
        ch.ChVector3d(0, -10, 0),
        Irrlicht.SColor(255, 255, 255, 255),
        1000.0,
    )

    
    ros_manager.Initialize()

    
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
            
            if not ros_manager.Update(time, time_step):
                break  

            
            driver.beginScene(True, True, Irrlicht.SColor(100, 100, 100, 255))
            scene_manager.drawAll()
            device.postEvent(Irrlicht.EEventType.EVENT_QUIT)
            driver.endScene()

        step_number += 1  
        realtime_timer.Spin(time_step)  


if __name__ == "__main__":
    main()