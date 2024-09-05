import pychrono as chrono
import pychrono.ros as chros
import rclpy
from rclpy.node import Node


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  


mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.5)  
mat.SetRestitution(0.01)  


floor = chrono.ChBodyEasyBox(10, 10, 1, 1000, True, True, mat)
floor.SetPos(chrono.ChVector3d(0, 0, -0.5))  
floor.SetFixed(True)  
system.Add(floor)  


box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, mat)
box.SetPos(chrono.ChVector3d(0, 0, 2))  
box.SetRot(chrono.QuatFromAngleZ(chrono.CH_PI / 4))  
system.Add(box)  


ros_manager = chros.ChROSPythonManager()
ros_manager.SetVerbose(True)  
ros_manager.Initialize()  


class IntPublisher(chros.ChROSHandler):
    def __init__(self, topic):
        super().__init__()  
        self.topic = topic  
        self.publisher = None  
        self.count = 0  

    def OnInit(self):
        
        self.publisher = self.GetNode().create_publisher(int, self.topic, 10)
        print(f"Publishing to topic: {self.topic}")

    def OnAdvance(self, time):
        
        msg = int(self.count)
        self.publisher.publish(msg)
        print(f"Published: {msg} to {self.topic}")
        self.count += 1  


int_pub_handler = IntPublisher("/chrono/test_int")
ros_manager.RegisterHandler(int_pub_handler)


ros_manager.RegisterClockHandler(chros.ChROSPythonClockHandler())
ros_manager.RegisterBodyHandler(chros.ChROSPythonBodyHandler(box, "~/box"))
ros_manager.RegisterTFHandler(chros.ChROSPythonTFHandler(box, floor, "~/box_frame", "~/floor_frame"))


time = 0
time_step = 1e-3  
time_end = 10  

while time < time_end:
    
    system.DoStepDynamics(time_step)
    time = system.GetChTime()  

    
    if not ros_manager.Update(time, time_step):
        break  


ros_manager.Shutdown()