import pychrono as ch
import pychrono.ros as chros
import pychrono.irrlicht as ch_irr  

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Int64


class MyCustomHandler(chros.ChROSHandler):
    

    def __init__(self, topic, rate):
        super().__init__(rate)  
        self.topic = topic
        self.publisher = None
        self.ticker = 0

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        
        print(f"Creating publisher for topic {self.topic} ...")
        
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, qos)
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
    floor.SetName("world")  
    sys.Add(floor)

    
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 5))
    box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))
    box.SetName("box")
    sys.Add(box)

    
    concrete_tex = ch.GetChronoDataFile("textures/concrete.jpg")
    cube_tex = ch.GetChronoDataFile("textures/cubetexture.png")
    floor.GetVisualShape(0).SetTexture(concrete_tex)
    box.GetVisualShape(0).SetTexture(cube_tex)

    
    vis = ch_irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("PyChrono ROS Simulation")
    vis.Initialize()
    vis.AddLogo(ch.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(ch.ChVector3d(3, 4, 5), ch.ChVector3d(0, 0, 1))
    vis.AddTypicalLights()
    vis.AddGrid(1.0, 1.0, 20, 20, 
                ch.ChCoordsysd(ch.ChVector3d(0, 0, -1), ch.QuatFromAngleX(ch.CH_PI_2)),
                ch.ChColor(0.2, 0.2, 0.2))

    
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    
    
    publish_rate = 10  
    
    
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(publish_rate, box, "~/box_state"))
    
    
    tf_handler = chros.ChROSTFHandler(publish_rate)
    tf_handler.AddTransform(box, floor, "box", "world")  
    ros_manager.RegisterHandler(tf_handler)
    
    
    custom_handler = MyCustomHandler("~/my_topic", publish_rate)
    ros_manager.RegisterPythonHandler(custom_handler)

    
    ros_manager.Initialize()

    
    time_step = 1e-3
    time_end = 30
    time = 0
    
    
    step_number = 0
    render_step_size = 1.0 / 50  
    render_steps = int(render_step_size / time_step)

    
    realtime_timer = ch.ChRealtimeStepTimer()

    
    while time < time_end:
        
        sys.DoStepDynamics(time_step)
        time = sys.GetChTime()
        step_number += 1

        
        if not ros_manager.Update(time, time_step):
            break

        
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        
        realtime_timer.Spin(time_step)

if __name__ == "__main__":
    main()