import pychrono as ch
import pychrono.ros as chros
import rclpy
from rclpy.publisher import Publisher
from std_msgs.msg import Int64
import time


class MyCustomHandler(chros.ChROSHandler):
    

    def __init__(self, topic, publish_rate=10):
        super().__init__(1)  
        self.topic = topic
        self.publisher: Publisher = None
        self.ticker = 0  
        self.publish_rate = publish_rate  

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
        time.sleep(self.publish_rate * 1e-6)  

def SetTexture(body: ch.ChBody, texture_path: str):
    
    body.SetTexture(ch.ChTexture2d(texture_path))

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

    
    renderer = ch.ChIrrRenderer(sys.GetName())
    renderer.SetCoordinateSystem(ch.ChVector3d(0, 0, 0), ch.ChVector3d(1, 0, 0), ch.ChVector3d(0, 1, 0))
    renderer.SetCameraDistance(5, 1)
    renderer.SetLightSetup(ch.ChLightSetup())
    renderer.GetLightSetup().Add(ch.ChLightPos(0, 0, 5, 0, 0, 0))
    renderer.GetLightSetup().EnableSkyRenderer(True)
    renderer.GetLightSetup().SetSkyRendererParameters(1024, 1024, 5, 16, 12)

    
    ros_manager = chros.ChROSPythonManager()
    
    
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    
    
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/box"))
    
    
    tf_handler = chros.ChROSTFHandler(30)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)
    
    
    custom_handler = MyCustomHandler("~/my_topic", 10)
    ros_manager.RegisterPythonHandler(custom_handler)

    
    ros_manager.Initialize()

    
    step_number = 0
    render_step_size = 100  
    render_steps = 10      

    
    import threading
    render_thread = threading.Thread(target=render_scene, args=(sys, renderer, render_step_size, render_steps))
    render_thread.daemon = True
    render_thread.start()

    
    time = 0
    time_step = 1e-3  
    time_end = 30  

    realtime_timer = ch.ChRealtimeStepTimer()  
    while time < time_end:
        sys.DoStepDynamics(time_step)  
        time = sys.GetChTime()  

        if not ros_manager.Update(time, time_step):  
            break  

        realtime_timer.Spin(time_step)  

        
        step_number += 1
        if step_number % render_step_size == 0:
            render_scene(sys, renderer, render_step_size, render_steps)

    
    render_thread.join()

def render_scene(sys, renderer, render_step_size, render_steps):
    
    import time
    last_render_time = time.time() * 1e-6
    while True:
        current_time = time.time() * 1e-6
        elapsed = current_time - last_render_time
        if elapsed >= (1 / render_steps) * 1e-6:
            renderer.Update(sys, ch.ChVector3d(current_time - sys.GetChTime()))
            last_render_time = current_time
            time.sleep(1 / render_steps)


if __name__ == "__main__":
    main()