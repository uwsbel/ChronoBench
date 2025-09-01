import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64


class MyCustomHandler(chros.ChROSHandler):
    

    def __init__(self, topic):
        super().__init__(1)  

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
    
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, -9.81))  

    
    phys_mat = chrono.ChContactMaterialNSC()
    phys_mat.SetFriction(0.5)  

    
    floor = chrono.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
    floor.SetPos(chrono.ChVectorD(0, 0, -1))  
    floor.SetFixed(True)  
    floor.SetName("base_link")  
    sys.Add(floor)  

    
    box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(chrono.ChVectorD(0, 0, 5))  
    box.SetRot(chrono.Q_from_AngX(0.2))  
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

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(0, 3, 6))
    vis.AddTypicalLights()
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Simulation Window')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))

    
    floor_shape = floor.GetVisualShape(0)
    floor_shape.SetTexture(chrono.GetChronoDataFile('floor_texture.jpg'))
    box_shape = box.GetVisualShape(0)
    box_shape.SetTexture(chrono.GetChronoDataFile('box_texture.jpg'))

    
    step_number = 0
    render_step_size = 10
    render_steps = 0

    
    time = 0
    time_step = 1e-3  
    time_end = 30  

    realtime_timer = chrono.ChRealtimeStepTimer()  
    while time < time_end:
        sys.DoStepDynamics(time_step)  
        time = sys.GetChTime()  

        if not ros_manager.Update(time, time_step):  
            break  

        
        render_steps += 1
        if render_steps >= render_step_size:
            vis.BeginScene()
            vis.DrawAll()
            vis.EndScene()
            render_steps = 0

        realtime_timer.Spin(time_step)  


if __name__ == "__main__":
    main()