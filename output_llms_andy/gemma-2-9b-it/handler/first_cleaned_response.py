import pychrono as chrono
import pychrono_ros  


chrono.SetChronoData(chrono.ChSystemNSC)  
system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.5)
material.SetRestitution(0.2)


floor = chrono.ChBodyEasyBox(10, 10, 0.1, material)
floor.SetPos(chrono.ChVectorD(0, 0, -0.1))
floor.SetBodyFixed(True)
system.AddBody(floor)


box = chrono.ChBodyEasyBox(1, 1, 1, material)
box.SetPos(chrono.ChVectorD(0, 0, 1))
system.AddBody(box)


ros = pychrono_ros.RosManager()
ros.Init()


class MyRosHandler(pychrono_ros.RosHandler):
    def __init__(self, topic):
        super().__init__()
        self.topic = topic

    def Update(self, system, step_size):
        
        position = box.GetPos()
        msg = {"x": int(position.x), "y": int(position.y), "z": int(position.z)}
        ros.Publish(self.topic, msg)


ros.RegisterBodyHandler(box)  
ros.RegisterClockHandler()
ros.RegisterTransformHandler()
ros.RegisterRosHandler(MyRosHandler("/my_topic"))  


while True:
    system.DoStepDynamics(0.01)  
    ros.Update()