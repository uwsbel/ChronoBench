import pychrono as ch
import pychrono.ros as chros
import pychrono.irrlicht as irr
from std_msgs.msg import Int64


class MyCustomHandler(chros.ChROSHandler):
    

    def __init__(self, topic):
        super().__init__(10)  

        self.topic = topic
        self.publisher: chros.ChROSHandler = None
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
    floor.SetTexture("floor_texture.png")  
    sys.Add(floor)  

    
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 5))  
    box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))  
    box.SetName("box")  
    box.SetTexture("box_texture.png")  
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

    
    application = irr.ChIrrApp(sys, "PyChrono Simulation", irr.dimension2d_u32(800, 600))
    application.AddCamera(irr.camera)
    application.AddLight(irr.light)
    application.SetCameraVerticalAlignment(irr.VerticalAlignment.VA_TOP)
    application.SetCameraPosition(ch.ChVector3d(0, 5, 10))
    application.SetCameraTarget(ch.ChVector3d(0, 0, 0))
    application.SetWindowSize(800, 600)
    application.SetWindowCaption("PyChrono Simulation with ROS and Irrlicht")
    application.SetMaxSpeed(50)

    
    step_number = 0
    render_step_size = 10
    render_steps = render_step_size

    
    time = 0
    time_step = 1e-3  
    time_end = 30  

    realtime_timer = ch.ChRealtimeStepTimer()  
    while time < time_end:
        sys.DoStepDynamics(time_step)  
        time = sys.GetChTime()  

        if not ros_manager.Update(time, time_step):  
            break  

        step_number += 1
        if step_number % render_steps == 0:
            application.BeginScene()
            application.DrawAll()
            application.EndScene()

        realtime_timer.Spin(time_step)  


if __name__ == "__main__":
    main()