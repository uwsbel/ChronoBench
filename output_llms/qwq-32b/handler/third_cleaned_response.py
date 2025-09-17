import pychrono as ch
import pychrono.ros as chros
import pychrono.irrlicht as irr
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

    
    my_vis = irr.ChIrrApp(sys, 'PyChrono Simulation', True)
    my_vis.SetCameraPosition(ch.ChVectorD(0, 3, 5))
    my_vis.SetCameraRotation(ch.ChVectorD(0, 45, 0))
    my_vis.AddTypicalLights()
    my_vis.AddLightWithDirection(ch.ChVectorD(0, 0, 1), ch.ChColor(1, 1, 1), 2)
    my_vis.AssetBind()
    my_vis.AssetUpdate()
    my_vis.SetWindowSize(1280, 1024)
    my_vis.SetWindowTitle('PyChrono ROS Simulation')

    
    floor_texture = ch.ChTexture()
    floor_texture.SetTextureFilename("textures/floor.jpg")
    floor.AddAsset(floor_texture)

    box_texture = ch.ChTexture()
    box_texture.SetTextureFilename("textures/box.jpg")
    box.AddAsset(box_texture)

    
    publish_rate = 10  
    ros_manager = chros.ChROSPythonManager()

    
    ros_manager.RegisterHandler(chros.ChROSClockHandler(publish_rate))
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(publish_rate, box, "~/box"))

    tf_handler = chros.ChROSTFHandler(publish_rate)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)

    
    custom_handler = MyCustomHandler("~/my_topic", publish_rate)
    ros_manager.RegisterPythonHandler(custom_handler)

    ros_manager.Initialize()

    
    time_step = 1e-3
    time_end = 30
    render_step_size = 0.01  
    render_steps = int(render_step_size / time_step)
    step_number = 0

    realtime_timer = ch.ChRealtimeStepTimer()

    while sys.GetChTime() < time_end:
        sys.DoStepDynamics(time_step)
        current_time = sys.GetChTime()

        
        if not ros_manager.Update(current_time, time_step):
            break

        
        step_number += 1
        if step_number % render_steps == 0:
            my_vis.BeginScene()
            my_vis.DrawAll()
            my_vis.EndScene()
            step_number = 0  

        realtime_timer.Spin(time_step)

if __name__ == "__main__":
    main()