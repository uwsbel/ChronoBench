import pychrono as ch
import pychrono.ros as chros
from std_msgs.msg import Int64


class MyCustomHandler(chros.ChROSHandler):
    

    def __init__(self, topic, rate=10):
        super().__init__(rate)  
        self.topic = topic
        self.publisher = None
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

    
    phys_mat = ch.ChMaterialSurfaceNSC()
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

    
    
    floor_shape = floor.GetVisualShape(0)
    floor_texture = ch.ChTexture()
    floor_texture.SetPath("textures/floor.jpg")
    floor_shape.SetTexture(floor_texture)

    box_shape = box.GetVisualShape(0)
    box_texture = ch.ChTexture()
    box_texture.SetPath("textures/box.jpg")
    box_shape.SetTexture(box_texture)

    
    my_vis = ch.ChIrrApp(sys, 'PyChrono ROS Demo', True)
    my_vis.SetWindowSize(1280, 1024)
    my_vis.SetWindowTitle('My Simulation')
    my_vis.SetCamPosition(ch.ChVectorD(0, 5, 10))
    my_vis.SetCamRotation(ch.Q_from_AngAxis(0.2, ch.ChVectorD(1, 0, 0)))
    my_vis.AddTypicalLights()
    my_vis.AllocateResources()
    my_vis.AssetBind()
    my_vis.AssetUpdate()
    my_vis.SetTimestep(1e-3)  

    
    publish_rate = 10  

    
    ros_manager = chros.ChROSPythonManager()

    
    clock_handler = chros.ChROSClockHandler(publish_rate)
    ros_manager.RegisterHandler(clock_handler)

    
    body_handler = chros.ChROSBodyHandler(publish_rate, box, "~/box")
    ros_manager.RegisterHandler(body_handler)

    
    tf_handler = chros.ChROSTFHandler(publish_rate)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)

    
    custom_handler = MyCustomHandler("~/my_topic", publish_rate)
    ros_manager.RegisterPythonHandler(custom_handler)

    
    ros_manager.Initialize()

    
    time_step = 1e-3  
    time_end = 30     
    render_steps = 10  
    step_number = 0

    
    realtime_timer = ch.ChRealtimeStepTimer()

    
    time = 0
    while time < time_end:
        sys.DoStepDynamics(time_step)
        time = sys.GetChTime()

        
        if not ros_manager.Update(time, time_step):
            break

        
        step_number += 1
        if step_number % render_steps == 0:
            my_vis.BeginScene()
            my_vis.DrawAll()
            my_vis.EndScene()

        
        realtime_timer.Spin(time_step)

    
    my_vis.Close()

if __name__ == "__main__":
    main()

import pychrono as ch
import pychrono.ros as chros
from std_msgs.msg import Int64

class MyCustomHandler(chros.ChROSHandler):
    
    def __init__(self, topic, rate=10):
        super().__init__(rate)
        self.topic = topic
        self.publisher = None
        self.ticker = 0

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        print(f"Creating publisher for {self.topic}...")
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 1)
        return True

    def Tick(self, time: float):
        print(f"Publishing {self.ticker}...")
        msg = Int64()
        msg.data = self.ticker
        self.publisher.publish(msg)
        self.ticker += 1

def main():
    sys = ch.ChSystemNSC()
    sys.SetGravitationalAcceleration(ch.ChVectorD(0, 0, -9.81))

    
    phys_mat = ch.ChMaterialSurfaceNSC()
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

    
    floor_shape = floor.GetVisualShape(0)
    floor_texture = ch.ChTexture()
    floor_texture.SetPath("textures/floor.jpg")
    floor_shape.SetTexture(floor_texture)

    box_shape = box.GetVisualShape(0)
    box_texture = ch.ChTexture()
    box_texture.SetPath("textures/box.jpg")
    box_shape.SetTexture(box_texture)

    
    my_vis = ch.ChIrrApp(sys, 'PyChrono ROS Demo', True)
    my_vis.SetWindowSize(1280, 1024)
    my_vis.SetWindowTitle('My Simulation')
    my_vis.SetCamPosition(ch.ChVectorD(0, 5, 10))
    my_vis.SetCamRotation(ch.Q_from_AngAxis(0.2, ch.ChVectorD(1, 0, 0)))
    my_vis.AddTypicalLights()
    my_vis.AllocateResources()
    my_vis.AssetBind()
    my_vis.AssetUpdate()
    my_vis.SetTimestep(1e-3)

    
    publish_rate = 10
    ros_manager = chros.ChROSPythonManager()

    
    clock_handler = chros.ChROSClockHandler(publish_rate)
    ros_manager.RegisterHandler(clock_handler)

    body_handler = chros.ChROSBodyHandler(publish_rate, box, "~/box")
    ros_manager.RegisterHandler(body_handler)

    tf_handler = chros.ChROSTFHandler(publish_rate)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)

    custom_handler = MyCustomHandler("~/my_topic", publish_rate)
    ros_manager.RegisterPythonHandler(custom_handler)

    ros_manager.Initialize()

    
    time_step = 1e-3
    time_end = 30
    render_steps = 10
    step_number = 0
    realtime_timer = ch.ChRealtimeStepTimer()

    time = 0
    while time < time_end:
        sys.DoStepDynamics(time_step)
        time = sys.GetChTime()
        if not ros_manager.Update(time, time_step):
            break

        step_number += 1
        if step_number % render_steps == 0:
            my_vis.BeginScene()
            my_vis.DrawAll()
            my_vis.EndScene()

        realtime_timer.Spin(time_step)

    my_vis.Close()

if __name__ == "__main__":
    main()