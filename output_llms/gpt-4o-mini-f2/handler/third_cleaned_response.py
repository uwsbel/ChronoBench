import pychrono as ch
import pychrono.ros as chros
import pychrono.visualization as viz
import rclpy.publisher
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
    
    sys = ch.ChSystemNSC()
    sys.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.81))  

    
    phys_mat = ch.ChContactMaterialNSC()
    phys_mat.SetFriction(0.5)  

    
    floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
    floor.SetPos(ch.ChVector3d(0, 0, -1))  
    floor.SetFixed(True)  
    floor.SetName("base_link")  
    floor.GetVisualShape(0).SetTexture(ch.GetChronoDataFile("textures/concrete.jpg"))
    sys.Add(floor)  

    
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 5))  
    box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))  
    box.SetName("box")  
    box.GetVisualShape(0).SetTexture(ch.GetChronoDataFile("textures/blue.png"))
    sys.Add(box)  
    
    
    sys.SetVisualizationType(ch.ChVisualization.VisualizationType_POVRay)
    
    ros_manager = chros.ChROSPythonManager()
    publish_rate = 10  
    
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    
    
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(publish_rate, box, "~/box"))
    
    
    tf_handler = chros.ChROSTFHandler(publish_rate)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)
    
    
    custom_handler = MyCustomHandler("~/my_topic")
    ros_manager.RegisterPythonHandler(custom_handler)

    
    ros_manager.Initialize()
    
    
    time = 0
    time_step = 1e-3  
    time_end = 30  
    
    step_number = 0
    render_step_size = 1.0 / 25  
    render_steps = int(render_step_size / time_step)
    
    ch.GetSystem().SetVisualizationType(ch.ChVisualization.VisualizationType_IRR)
    viz.ChVisualSystemIrrlicht().AttachSystem(sys).SetWindowTitle(
        "Viper rover - Rigid terrain").SetWindowSize(1280, 720).SetChaseCamera(ch.vector3df(0, 0, 2), ch.vector3df(0, 0, 0)
    ).Initialize()
    realtime_timer = ch.ChRealtimeStepTimer()  
    while time < time_end:
        sys.DoStepDynamics(time_step)  
        time = sys.GetChTime()  
        
        viz.GetSystem().GetSceneManager().drawAll()
        viz.BeginScene()
        viz.Render()
        viz.EndScene()
        if not ros_manager.Update(time, time_step):  
            break  

        realtime_timer.Spin(time_step)  
        step_number += 1

if __name__ == "__main__":
    main()