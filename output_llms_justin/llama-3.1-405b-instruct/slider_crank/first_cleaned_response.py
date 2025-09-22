import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


time_step = 0.001
time_end = 10


sys = chrono.ChSystemNSC()


application = chronoirr.ChIrrApp(sys, "PyChrono Crank-Slider Simulation", chronoirr.dimension2du(800, 600))


floor = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, True)
floor.SetPos(chrono.ChVectorD(0, -5, 0))
sys.AddBody(floor)


crankshaft = chrono.ChBodyEasyCylinder(sys, 1, 5, 1000, True, True)
crankshaft.SetPos(chrono.ChVectorD(0, 0, 0))
sys.AddBody(crankshaft)


connecting_rod = chrono.ChBodyEasyBox(sys, 5, 0.5, 0.5, 100, True, True)
connecting_rod.SetPos(chrono.ChVectorD(2.5, 0, 0))
sys.AddBody(connecting_rod)


piston = chrono.ChBodyEasyBox(sys, 1, 1, 1, 100, True, True)
piston.SetPos(chrono.ChVectorD(5, 0, 0))
sys.AddBody(piston)


revolute_joint = chrono.ChLinkRevolute()
revolute_joint.Initialize(crankshaft, connecting_rod, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_ROTATE_Y_TO_Z))
sys.AddLink(revolute_joint)

prismatic_joint = chrono.ChLinkPrismatic()
prismatic_joint.Initialize(connecting_rod, piston, chrono.ChFrameD(chrono.ChVectorD(2.5, 0, 0), chrono.Q_ROTATE_Y_TO_Z), chrono.ChVectorD(1, 0, 0))
sys.AddLink(prismatic_joint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crankshaft, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_ROTATE_Y_TO_Z))
motor.SetMotorFunction(chrono.ChFunction_Const(chrono.CH_C_ANG_SPEED, 10))
sys.AddLink(motor)


camera = application.GetSceneManager().GetActiveCamera()
camera.SetPosition(chrono.ChVectorD(0, 5, -10))
camera.SetTarget(chrono.ChVectorD(0, 0, 0))


light = application.GetSceneManager().GetLight(0)
light.SetPosition(chrono.ChVectorD(0, 5, -10))
light.SetTarget(chrono.ChVectorD(0, 0, 0))


application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()


application.SetTimestep(time_step)
application.SetTryRealtime(True)
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()
    if application.GetSystem().GetChTime() > time_end:
        break