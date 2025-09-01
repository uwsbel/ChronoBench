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
system.Add(floor)


box = chrono.ChBodyEasyBox(1, 1, 1, material)
box.SetPos(chrono.ChVectorD(0, 0, 1))
system.Add(box)


ros_manager = pychrono_ros.RosManager()
ros_manager.SetChronoSystem(system)


class MyRosHandler(pychrono_ros.RosHandler):
    def __init__(self, topic_name):
        super().__init__()
        self.topic_name = topic_name

    def Update(self, time_step):
        
        message = {"data": int(time_step * 10)}  
        self.publisher.publish(message)


ros_manager.RegisterHandler(MyRosHandler("my_topic"))


ros_manager.SetClockHandler(pychrono_ros.RosClockHandler())
ros_manager.SetBodyHandler(pychrono_ros.RosBodyHandler())
ros_manager.SetTransformHandler(pychrono_ros.RosTransformHandler())


while True:
    system.DoStepDynamics(0.01)  
    ros_manager.Update()