import pychrono as chrono
from pychrono import irrlicht
from pychrono.irrlicht import ChVisualSystemIrrlicht


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, False)
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0, -0.05, 0))
system.AddBody(ground)


crankshaft = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, 0.05, 0.2, 1000, True, False)
crankshaft.SetPos(chrono.ChVector3d(0, 0.1, 0))
crankshaft.SetName("Crankshaft")
system.AddBody(crankshaft)


connecting_rod = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, 0.03, 0.4, 1000, True, False)
connecting_rod.SetPos(chrono.ChVector3d(0.15, 0.1, 0))
connecting_rod.SetName("Connecting Rod")
system.AddBody(connecting_rod)


piston = chrono.ChBodyEasyBox(0.2, 0.05, 0.05, 1000, True, False)
piston.SetPos(chrono.ChVector3d(0.3, 0.1, 0))
piston.SetName("Piston")
system.AddBody(piston)


revolute_joint = chrono.ChLinkRevolute()
revolute_joint.SetName("Crankshaft-Connecting Rod Joint")
revolute_joint.Initialize(crankshaft, connecting_rod, chrono.ChFramed(chrono.ChVector3d(0.15, 0.1, 0), chrono.QUNIT))
system.AddLink(revolute_joint)


prismatic_joint = chrono.ChLinkPrismatic()
prismatic_joint.SetName("Connecting Rod-Piston Joint")
prismatic_joint.Initialize(connecting_rod, piston, chrono.ChFramed(chrono.ChVector3d(0.3, 0.1, 0), chrono.QUNIT))
prismatic_joint.SetCoordDirection(chrono.ChVector3d(1, 0, 0))
system.AddLink(prismatic_joint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.SetName("Crankshaft Motor")
motor.Initialize(crankshaft, ground, chrono.ChFramed(chrono.ChVector3d(0, 0.1, 0), chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunctionConst(2 * chrono.CH_C_PI))  


system.AddLink(motor)


vis = ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank-Slider Mechanism')
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, 2), chrono.ChVector3d(0, 0.1, 0))
vis.AddTypicalLights()


texture = chrono.ChTexture()
texture.SetFilename('path/to/texture.jpg')  
crankshaft.GetVisualShape(0).AddTexture(texture)


time_step = 1e-3
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)