import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono_ros as chronoros


chrono.SetChronoDataPath('./')


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.4)
material.SetDampingF(0.0001)
material.SetCompliance(0.0001)
material.SetComplianceT(0.0001)


floor = chrono.ChBodyEasyBox(system, 10, 1, 10, 1000, True, material)
floor.SetPos(chrono.ChVectorD(0, -1, 0))
floor.SetBodyFixed(True)
system.Add(floor)


box = chrono.ChBodyEasyBox(system, 1, 1, 1, 1000, True, material)
box.SetPos(chrono.ChVectorD(0, 2, 0))
system.Add(box)


class CustomROSPublisher(chronoros.ChROSHandler):
    def __init__(self):
        super().__init__()
        self.publisher = chronoros.ChROSNode().advertise('custom_topic', 'std_msgs/Int32', 10)

    def Update(self, time):
        msg = chronoros.Std_msgsInt32()
        msg.data = int(time)
        self.publisher.publish(msg)


ros_manager = chronoros.ChROSManager()
ros_manager.RegisterHandler(chronoros.ChROSClockHandler())
ros_manager.RegisterHandler(chronoros.ChROSBodyHandler())
ros_manager.RegisterHandler(chronoros.ChROSTransformHandler())
ros_manager.RegisterHandler(CustomROSPublisher())


while True:
    time = system.GetChTime()
    system.DoStepDynamics(0.01)
    ros_manager.Update(time)
    if time > 10:
        break


application = chronoirr.ChVisualSystemIrrlicht()
application.SetWindowSize(1024, 768)
application.SetWindowTitle('PyChrono Simulation')
application.SetTypicalSize(chrono.ChVectorD(1, 1, 1))
application.AddTypicalCamera(chrono.ChVectorD(0, 0, 5))
application.AddLightWithTarget(chrono.ChVectorD(2, 2, 2), chrono.ChVectorD(0, 0, 0))
application.AddCameraMotionMode(chronoirr.CAMERAMOTIONMODE_FOLLOWPATH)
application.AddCameraPathCurve(chrono.ChVectorD(0, 0, 5), chrono.ChVectorD(0, 0, 0), 1000)
application.AddSkyBox()
application.AddTypicalLigths()
application.AddPostprocessEffect(chronoirr.ChPostprocessEffect(chronoirr.ChPostprocessEffect.ANTIALIASING, 2))
application.SetShowFPS(True)
while application.Run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()