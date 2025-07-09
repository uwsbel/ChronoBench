import pychrono as chrono
import pychrono.irrlicht as chronoirr
import rospy
from std_msgs.msg import Int32


sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVector3d(0, -9.81, 0))  


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.5)


floor = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True, material)
floor.SetPos(chrono.ChVector3d(0, -5, 0))
floor.SetBodyFixed(True)
sys.Add(floor)


box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, material)
box.SetPos(chrono.ChVector3d(0, 0, 0))
sys.Add(box)


rospy.init_node('chrono_ros_node', anonymous=True)


class CustomROSHandler:
    def __init__(self, topic_name):
        self.publisher = rospy.Publisher(topic_name, Int32, queue_size=10)
        self.count = 0

    def update(self):
        msg = Int32()
        msg.data = self.count
        self.publisher.publish(msg)
        self.count += 1

custom_handler = CustomROSHandler('/chrono_int_topic')


class ROSManager:
    def __init__(self, sys):
        self.sys = sys
        self.clock_handler = chrono.ChROSClockHandler()
        self.body_handler = chrono.ChROSBodiesHandler(sys)
        self.transform_handler = chrono.ChROSTransformsHandler(sys)
        self.handlers = [self.clock_handler, self.body_handler, self.transform_handler, custom_handler]

    def update(self):
        for handler in self.handlers:
            if hasattr(handler, 'update'):
                handler.update()

ros_manager = ROSManager(sys)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Chrono Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    sys.DoStepDynamics(0.01)
    ros_manager.update()
    vis.EndScene()
    rospy.sleep(0.01)  


rospy.signal_shutdown('Simulation finished')