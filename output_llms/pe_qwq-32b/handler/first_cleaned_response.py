import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono_ros as chronoros
import rospy
from std_msgs.msg import Int32

class MyROSPublisher(chronoros.ChRosHandler):
    def __init__(self, topic, box):
        super().__init__()
        self.publisher = rospy.Publisher(topic, Int32, queue_size=10)
        self.box = box

    def Update(self):
        box_pos = self.box.GetPos()
        y_pos = int(box_pos.y)
        msg = Int32()
        msg.data = y_pos
        self.publisher.publish(msg)

def main():
    
    rospy.init_node('chrono_ros_node')

    
    my_system = chrono.ChSystemNSC()
    my_system.SetGravity(chrono.ChVectorD(0, -9.81, 0))

    
    floor = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True)
    floor.SetPos(chrono.ChVectorD(0, -1, 0))
    floor.SetBodyFixed(True)
    my_system.Add(floor)

    
    box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True)
    box.SetPos(chrono.ChVectorD(0, 1, 0))
    my_system.Add(box)

    
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(0.5)
    material.SetRestitution(0.1)
    floor.GetContactMaterial(0).SetMaterial(material)
    box.GetContactMaterial(0).SetMaterial(material)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(my_system)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('PyChrono ROS Simulation')
    vis.Initialize()
    vis.AddCamera(chronoirr.ChVectorD(0, 3, 6))
    vis.AddTypicalLights()
    vis.AddSkyBox()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.Render()

    
    manager = chronoros.ChRosManager(my_system)

    
    clock_handler = chronoros.ChRosClockHandler(my_system)
    body_handler = chronoros.ChRosBodyHandler(my_system)
    transform_handler = chronoros.ChRosTransformHandler(my_system)

    
    custom_handler = MyROSPublisher('/box_height', box)

    
    manager.AddHandler(clock_handler)
    manager.AddHandler(body_handler)
    manager.AddHandler(transform_handler)
    manager.AddHandler(custom_handler)

    
    time_step = 0.01  
    while vis.Run() and not rospy.is_shutdown():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        my_system.DoStepDynamics(time_step)

        
        manager.DoStepDynamics()

    
    vis.Close()
    rospy.signal_shutdown('Simulation ended')

if __name__ == '__main__':
    main()