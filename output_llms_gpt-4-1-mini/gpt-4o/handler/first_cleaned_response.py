import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono_ros as chronoros
import rospy
from std_msgs.msg import Int32


rospy.init_node('chrono_ros_simulation')


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.5)
material.SetDampingF(0.2)
material.SetCompliance(0.0001)


floor = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True, material)
floor.SetPos(chrono.ChVectorD(0, -1, 0))
floor.SetBodyFixed(True)
system.Add(floor)


box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, material)
box.SetPos(chrono.ChVectorD(0, 0.5, 0))
system.Add(box)


class CustomROSHandler(chronoros.ChROSHandler):
    def __init__(self, topic_name):
        super().__init__()
        self.publisher = rospy.Publisher(topic_name, Int32, queue_size=10)
        self.counter = 0

    def Update(self, time):
        msg = Int32()
        msg.data = self.counter
        self.publisher.publish(msg)
        self.counter += 1


ros_manager = chronoros.ChROSManager(system)


ros_manager.AddClockHandler()
ros_manager.AddBodyHandler(box, "box")
ros_manager.AddTransformHandler(box, "box_transform")


custom_handler = CustomROSHandler('custom_topic')
ros_manager.AddCustomHandler(custom_handler)


application = chronoirr.ChIrrApp(system, 'PyChrono ROS Simulation', chronoirr.dimension2du(800, 600))
application.AddTypicalSky()
application.AddTypicalLogo()
application.AddTypicalCamera(chronoirr.vector3df(0, 2, -6))
application.AddLightWithShadow(chronoirr.vector3df(2, 4, -2), chronoirr.vector3df(0, 0, 0), 9, 1, 9, 60)


application.AssetBindAll()
application.AssetUpdateAll()


timestep = 0.01
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    ros_manager.Update()
    application.EndScene()
    system.DoStepDynamics(timestep)
    rospy.sleep(timestep)  


rospy.signal_shutdown('Simulation finished')