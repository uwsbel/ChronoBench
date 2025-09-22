import pychrono as chrono
import pychrono.irrlicht as irr
import math


ch = chrono.ChEngine('SCN')
ch.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
ch.SetSolverType(chrono.ChSolver.Type_PENDANT)
ch.SetSolver(Iters=50)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(ch)
vis.SetWindowTitle('Crank-Slider Mechanism')
vis.SetWindowSize(800, 600)
vis.SetCameraVertical(chrono.ChVector3d(0, 0, 1))
vis.SetCameraDistance(2.5)
vis.SetLight(chrono.ChVector3d(0.5, 0.5, 0.5))
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))


floor = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True, vis.GetMaterial(0))
floor.SetPos(chrono.ChVector3d(0, -0.2, 0))
floor.SetFixed(True)
ch.Add(floor)


crankshaft = chrono.ChBodyEasyCylinder(0.05, 0.2, 1000, True, True, vis.GetMaterial(0))
crankshaft.SetPos(chrono.ChVector3d(0, 0.2, 0))
crankshaft.SetFixed(False)
crankshaft.SetMass(1)
crankshaft.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
ch.Add(crankshaft)


connecting_rod = chrono.ChBodyEasyCylinder(0.05, 0.5, 1000, True, True, vis.GetMaterial(0))
connecting_rod.SetPos(chrono.ChVector3d(0.2, 0.2, 0))
connecting_rod.SetFixed(False)
connecting_rod.SetMass(1)
connecting_rod.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
ch.Add(connecting_rod)


piston = chrono.ChBodyEasyCylinder(0.1, 0.1, 1000, True, True, vis.GetMaterial(0))
piston.SetPos(chrono.ChVector3d(0.4, 0.2, 0))
piston.SetFixed(False)
piston.SetMass(1)
piston.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
ch.Add(piston)


crankshaft_joint = chrono.ChLinkLockPointLine()
crankshaft_joint.Initialize(floor, crankshaft, chrono.ChFrame3d(chrono.ChVector3d(0, 0.2, 0), chrono.ChQuaterniond(1, 0, 0, 0)))
ch.Add(crankshaft_joint)

connecting_rod_joint1 = chrono.ChLinkLockSpherical()
connecting_rod_joint1.Initialize(crankshaft, connecting_rod, chrono.ChFrame3d(chrono.ChVector3d(0.1, 0.2, 0), chrono.ChQuaterniond(1, 0, 0, 0)))
ch.Add(connecting_rod_joint1)

connecting_rod_joint2 = chrono.ChLinkLockUniversal()
connecting_rod_joint2.Initialize(connecting_rod, piston, chrono.ChFrame3d(chrono.ChVector3d(0, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0)))
ch.Add(connecting_rod_joint2)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(floor, crankshaft, chrono.ChFrame3d(chrono.ChVector3d(0, 0.2, 0), chrono.ChQuaterniond(1, 0, 0, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(6))
ch.Add(motor)


vis.Initialize()
vis.AddTypicalLights()
vis.AddTypicalLogo()
vis.AddTypicalCamera(chrono.ChVector3d(0, -0.5, 0.5), chrono.ChVector3d(0, 0, 0), 45)


while vis.Run() and not vis.WasUserQuit():
    ch.DoStepSimulation(1e-3)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()