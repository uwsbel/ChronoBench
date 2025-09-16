import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import rospy
from std_msgs.msg import Int32


my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


my_material = chrono.ChMaterialSurfaceNSC()
my_material.SetFriction(0.4)
my_material.SetRestitution(0.01)
my_material.SetAdhesion(0)


floor_geometry = chrono.ChBoxGeometry(chrono.ChVectorD(10, 1, 10))
floor_body = chrono.ChBodyEasyBox(1, 0.05, 1, 100, True, True, my_material)
floor_body.SetBodyFixed(True)
floor_body.SetPos(chrono.ChVectorD(0, -0.05, 0))
my_system.Add(floor_body)


box_geometry = chrono.ChBoxGeometry(chrono.ChVectorD(0.5, 0.5, 0.5))
box_body = chrono.ChBodyEasyBox(1, 1, 1, 10, True, True, my_material)
box_body.SetPos(chrono.ChVectorD(0, 2, 0))
box_body.SetPos_dt(chrono.ChVectorD(0, -1, 0))
box_body.SetWvel_loc(chrono.ChVectorD(0.1, 0.1, 0.1))
my_system.Add(box_body)


class CustomROSHandler(object):
    def __init__(self):
        self.publisher = rospy.Publisher('/my_topic', Int32, queue_size=10)

    def publish(self, msg):
        self.publisher.publish(msg)


ros_manager = chrono.ChRosSystem(my_system)
ros_manager.Initialize(rospy.get_name())
ros_manager.AddHandler(chrono.ChRosClock(ros_manager))
ros_manager.AddHandler(chrono.ChRosBody(ros_manager, box_body))
ros_manager.AddHandler(chrono.ChRosTransform(ros_manager, box_body))
custom_handler = CustomROSHandler()
ros_manager.AddHandler(custom_handler)


my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
my_system.SetSolverMaxIterations(100)
my_system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

irrlicht_application = chronoirr.ChIrrApp(my_system, 'PyChrono ROS Integration', chronoirr.dimension_t(800, 600), False, False)
irrlicht_application.AddTypicalLogo(chronoirr.GetChronoDataFile('logo_pychrono_alpha.png'))
irrlicht_application.AddTypicalSky()
irrlicht_application.AddTypicalLights()
irrlicht_application.AddTypicalCamera(chronoirr.vector3df(0, 4, -5))

ros_manager.SetIrrlichtApplication(irrlicht_application)

while irrlicht_application.GetDevice().run():
    irrlicht_application.BeginScene()
    irrlicht_application.DrawAll()
    irrlicht_application.DoStep()
    irrlicht_application.EndScene()

    ros_manager.Update()

    custom_handler.publish(Int32(1))  

    chrono.ChCollisionSystemBullet.CollideAll()
    my_system.DoFrameDynamics(0.01)

    irrlicht_application.SynchronizePhysicsToGUI()

irrlicht_application.GetDevice().closeDevice()