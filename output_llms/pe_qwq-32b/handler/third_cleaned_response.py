import pychrono as ch
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros
import rclpy.publisher
from std_msgs.msg import Int64


class MyCustomHandler(chros.ChROSHandler):
    

    def __init__(self, topic, rate=10):
        super().__init__(rate)  
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
    sys.SetGravitationalAcceleration(ch.ChVectorD(0, 0, -9.81))  

    
    phys_mat = ch.ChContactMaterialNSC()
    phys_mat.SetFriction(0.5)

    
    floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
    floor.SetPos(ch.ChVectorD(0, 0, -1))  
    floor.SetFixed(True)
    floor.SetName("base_link")
    sys.Add(floor)

    
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVectorD(0, 0, 5))
    box.SetRot(ch.Q_from_AngAxis(0.2, ch.ChVectorD(1, 0, 0)))  
    box.SetName("box")
    sys.Add(box)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1280, 1024)
    vis.SetWindowTitle('ROS-PyChrono Simulation')
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chronoirr.ChVectorD(0, 3, 10))  
    vis.AddTypicalLights()
    vis.Initialize()

    
    floor_texture = chrono.GetChronoDataFile('textures/concrete.jpg')
    box_texture = chrono.GetChronoDataFile('textures/wood.jpg')
    floor.GetVisualShape(0).SetTexture(floor_texture)
    box.GetVisualShape(0).SetTexture(box_texture)

    
    publish_rate = 10  

    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler(publish_rate))
    
    
    ros_body_handler = chros.ChROSBodyHandler(publish_rate, box, "~/box")
    ros_manager.RegisterHandler(ros_body_handler)
    
    
    tf_handler = chros.ChROSTFHandler(publish_rate)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)
    
    
    custom_handler = MyCustomHandler("~/my_topic", publish_rate)
    ros_manager.RegisterPythonHandler(custom_handler)

    ros_manager.Initialize()

    
    time_step = 0.001
    time_end = 30.0
    step_number = 0
    render_step_size = 0.033  
    render_steps = int(render_step_size / time_step)
    realtime_timer = ch.ChRealtimeStepTimer()

    time = 0.0
    while time < time_end and vis.Run():
        sys.DoStepDynamics(time_step)
        time = sys.GetChTime()

        if not ros_manager.Update(time, time_step):
            break

        
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
        step_number += 1

        realtime_timer.Spin(time_step)

if __name__ == "__main__":
    main()