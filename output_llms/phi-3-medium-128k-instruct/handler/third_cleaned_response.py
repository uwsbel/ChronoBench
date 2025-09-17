import pychrono as ch
import pychrono.ros as chros
import rclpy.publisher
from std_msgs.msg import Int64


class MyCustomHandler(chros.ChROSHandler):
    

    def __init__(self, topic):
        super().__init__(10)  
        self.topic = topic
        self.publisher = None
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

    
    irr_options = ch.ChIrrAppearance()
    irr_options.SetTexture(ch.ChIrrAppearance.TEXTURE_FILE_PLAIN_RGB, "path/to/floor_texture.png")
    floor.SetAppearance(irr_options)
    irr_options.SetTexture(ch.ChIrrAppearance.TEXTURE_FILE_PLAIN_RGB, "path/to/box_texture.png")
    box.SetAppearance(irr_options)

    
    irr_renderer = ch.ChIrrAppearance()
    irr_renderer.SetCamera(ch.ChIrrAppearance.CAMERA_PERSPECTIVE)
    irr_renderer.SetLighting(ch.ChIrrAppearance.LIGHTING_ALL)
    irr_renderer.SetWindowProperties(ch.ChIrrAppearance.WINDOW_WIDTH, 800, ch.ChIrrAppearance.WINDOW_HEIGHT, 600, ch.ChIrrAppearance.WINDOW_TITLE, "My Simulation")
    irr_renderer.AddCamera(ch.ChIrrAppearance.CAMERA_DEFAULT, ch.ChIrrAppearance.CAMERA_POSITION, ch.ChVector3d(0, 0, 10))
    irr_renderer.AddCamera(ch.ChIrrAppearance.CAMERA_RIGHT, ch.ChIrrAppearance.CAMERA_POSITION, ch.ChVector3d(10, 0, 0))
    irr_renderer.AddLight(ch.ChIrrAppearance.LIGHT_POSITION, ch.ChVector3d(0, 0, 10))
    irr_renderer.AddLight(ch.ChIrrAppearance.LIGHT_POSITION, ch.ChVector3d(0, 10, 0))
    irr_renderer.AddLight(ch.ChIrrAppearance.LIGHT_POSITION, ch.ChVector3d(0, -10, 0))
    irr_renderer.AddLight(ch.ChIrrAppearance.LIGHT_POSITION, ch.ChVector3d(-10, 0, 0))
    irr_renderer.AddLight(ch.ChIrrAppearance.LIGHT_POSITION, ch.ChVector3d(0, 0, -10))

    
    irr_window = ch.ChIrrWindow()
    irr_window.SetWindowTitle("My Simulation")
    irr_window.SetWindowSize(irr_renderer.GetWindowWidth(), irr_renderer.GetWindowHeight())
    irr_window.SetWindowPosition(0, 0)
    irr_window.AddRenderer(irr_renderer)

    
    step_number = 0
    render_step_size = 1e-3  
    render_steps = 10  

    
    time = 0
    time_end = 30  
    time_step = 1e-3  
    while time < time_end:
        sys.DoStepDynamics(time_step)  
        time = sys.GetChTime()  

        if not ros_manager.Update(time, time_step):  
            break  

        if step_number % render_steps == 0:
            irr_window.Render()  
        step_number += 1

        realtime_timer = ch.ChRealtimeStepTimer()  
        realtime_timer.Spin(time_step)  


if __name__ == "__main__":
    main()