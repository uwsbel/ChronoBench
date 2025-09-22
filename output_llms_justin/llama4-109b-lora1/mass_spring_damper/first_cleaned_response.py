import pychrono as chrono
import pychrono.irrlicht as irr
import math


chrono.SetChairsThreading(chrono.GetNumProcs())
chrono.SetChronoDataPath(chrono.GetChronoDataPath())


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowTitle('PyChrono demo')
vis.SetWindowSize(800, 600)
vis.SetRenderMode(irr.RenderMode_VITRUAL)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 0.2), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()


ground = chrono.ChBody()
ground.SetFixed(True)
my_system.Add(ground)


mass = chrono.ChBody()
mass.SetMass(1)
mass.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
mass.SetPos(chrono.ChVector3d(0, 0, 0.5))
my_system.Add(mass)


mass_shape = chrono.ChSphereShape(0.1)
mass.AddVisualShape(mass_shape)


link = chrono.ChLinkTSDA()
link.Initialize(ground, mass, False, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0.5), False, False)
link.SetRestLength(0.5)
link.SetSpringCoefficient(1000)
link.SetDampingCoefficient(10)
link.SetForceMode(chrono.LinkForceMode_SPRING_AND_DAMPER)
my_system.Add(link)


link.AddVisualShapeSpring(0.06, 12, chrono.ChColor(0.6, 0.6, 0.6))


my_system.SetTimestep(0.01)
my_system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


while vis.Run() :
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    my_system.DoStepDynamics(0.01)