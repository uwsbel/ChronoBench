import pychrono as ch
import pychrono.irrlicht as irr
import pychrono.ros as chros

import rclpy
from std_msgs.msg import Int64




class MyCustomHandler(chros.ChROSHandler):
    def __init__(self, topic: str, publish_rate: float):
        
        super().__init__(publish_rate)
        self.topic = topic
        self.publisher = None
        self.ticker = 0

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        
        print(f"[MyCustomHandler] Creating publisher on '{self.topic}' ...")
        node = interface.GetNode()
        self.publisher = node.create_publisher(Int64, self.topic, 1)
        return True

    def Tick(self, time: float):
        
        msg = Int64()
        msg.data = self.ticker
        print(f"[MyCustomHandler] Publishing {msg.data}")
        self.publisher.publish(msg)
        self.ticker += 1


def main():
    
    rclpy.init()

    
    sys = ch.ChSystemNSC()
    sys.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.81))

    
    mat = ch.ChMaterialSurfaceNSC()
    mat.SetFriction(0.5)

    
    floor = ch.ChBodyEasyBox(
        10, 10, 1,        
        1000,             
        True, True, mat   
    )
    floor.SetPos(ch.ChVector3d(0, 0, -1))
    floor.SetFixed(True)
    floor.SetName("base_link")
    
    floor.SetTexture(ch.GetChronoDataFile("textures/concrete.jpg"))
    sys.Add(floor)

    
    box = ch.ChBodyEasyBox(
        1, 1, 1,
        1000,
        True, True, mat
    )
    box.SetPos(ch.ChVector3d(0, 0, 5))
    
    box.SetRot(ch.Q_from_AngAxis(0.2, ch.ChVector3d(1, 0, 0)))
    box.SetName("box")
    box.SetTexture(ch.GetChronoDataFile("textures/wood.jpg"))
    sys.Add(box)

    
    app = irr.ChIrrApp(
        sys,
        "PyChrono + ROS2 + Irrlicht Demo",
        irr.dimension2du(1024, 768)
    )
    app.AddTypicalSky()
    app.AddTypicalCamera(
        irr.vector3df(0, -10, 5),  
        irr.vector3df(0, 0, 0)     
    )
    app.AddTypicalLights()

    
    app.AssetBindAll()
    app.AssetUpdateAll()

    
    publish_rate = 10.0  

    ros_manager = chros.ChROSPythonManager()

    
    ros_manager.RegisterHandler(chros.ChROSClockHandler(publish_rate))

    
    ros_manager.RegisterHandler(
        chros.ChROSBodyHandler(publish_rate, box, "~/box")
    )

    
    tfh = chros.ChROSTFHandler(publish_rate)
    tfh.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tfh)

    
    custom_h = MyCustomHandler("~/my_topic", publish_rate)
    ros_manager.RegisterHandler(custom_h)

    ros_manager.Initialize()

    
    time       = 0.0
    time_step  = 1e-3   
    time_end   = 30.0   
    step_number       = 0
    render_frame_rate = 30.0                           
    render_steps      = int(1.0 / time_step / render_frame_rate)

    realtime_timer = ch.ChRealtimeStepTimer()

    
    while app.GetDevice().run() and time < time_end:
        
        sys.DoStepDynamics(time_step)
        time = sys.GetChTime()

        
        if not ros_manager.Update(time, time_step):
            print("[main] ROS manager signalled an error, shutting down.")
            break

        
        if step_number % render_steps == 0:
            app.BeginScene(True, True, irr.SColor(255, 100, 100, 140))
            app.DrawAll()
            app.EndScene()

        realtime_timer.Spin(time_step)
        step_number += 1

    
    rclpy.shutdown()


if __name__ == "__main__":
    main()