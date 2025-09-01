vis = chvis.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Simulation Window')
vis.AddLogo(ch.GetChronoDataFile('pychrono.png'))
vis.AddSkyBox()
cam_pos = ch.ChVector3d(0, 3, 6)
vis.AddCamera(cam_pos)
vis.AddTypicalLights()


floor_shape = ch.ChVisualShapeBox(10, 10, 1)
floor_shape.SetTexture(ch.GetChronoDataFile('floor_texture.png'))
floor.AddVisualShape(floor_shape)

box_shape = ch.ChVisualShapeBox(1, 1, 1)
box_shape.SetTexture(ch.GetChronoDataFile('box_texture.png'))
box.AddVisualShape(box_shape)


step_number = 0
render_step_size = 10  
render_steps = 100  

while time < time_end:
    sys.DoStepDynamics(time_step)
    time = sys.GetChTime()

    
    if step_number % render_step_size == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        step_number += 1

    if not ros_manager.Update(time, time_step):
        break

import pychrono as ch
import pychrono.ros as chros

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
    sys.Add(floor)  

    
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 5))  
    box.SetRot(ch.QuatFromAngleAxis(0.2, ch.ChVector3d(1, 0, 0)))  
    box.SetName("box")  
    sys.Add(box)  

    
    vis = chvis.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Simulation Window')
    vis.AddLogo(ch.GetChronoDataFile('pychrono.png'))
    vis.AddSkyBox()
    cam_pos = ch.ChVector3d(0, 3, 6)
    vis.AddCamera(cam_pos)
    vis.AddTypicalLights()

    
    floor_shape = ch.ChVisualShapeBox(10, 10, 1)
    floor_shape.SetTexture(ch.GetChronoDataFile('floor_texture.png'))
    floor.AddVisualShape(floor_shape)

    box_shape = ch.ChVisualShapeBox(1, 1, 1)
    box_shape.SetTexture(ch.GetChronoDataFile('box_texture.png'))
    box.AddVisualShape(box_shape)

    
    ros_manager = chros.ChROSPythonManager()
    
    
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    
    
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/box"))
    
    
    tf_handler = chros.ChROSTFHandler(30)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)
    
    
    custom_handler = MyCustomHandler("~/my_topic")
    ros_manager.RegisterPythonHandler(custom_handler)

    
    ros_manager.Initialize()

    
    time = 0
    time_step = 1e-3  
    time_end = 30  

    
    step_number = 0
    render_step_size = 10  
    render_steps = 100  

    while time < time_end:
        sys.DoStepDynamics(time_step)
        time = sys.GetChTime()  

        
        if step_number % render_step_size == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            step_number += 1

        if not ros_manager.Update(time, time_step):  
            break  


if __name__ == "__main__":
    main()