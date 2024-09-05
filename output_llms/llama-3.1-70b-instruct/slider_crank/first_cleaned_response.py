import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.engine as chronoengine


mysystem = chrono.ChSystemNSC()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Crank-Slider Mechanism")
vis.SetCamera(chrono.ChVectorD(0, 0, 2), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
vis.SetTypicalLenght(1.0)
vis.SetCameraVertical(chrono.ChVectorD(0, 1, 0))
vis.SetCameraUp(chrono.ChVectorD(0, 0, 1))
vis.SetCameraSpeed(0.5)
vis.SetCameraRotSpeed(0.5)


floor = chrono.ChBodyEasyBox(mysystem, 4, 0.1, 4, 1000, True, True)
floor.SetPos(chrono.ChVectorD(0, -1, 0))
floor.SetBodyFixed(True)
mysystem.Add(floor)


crankshaft = chrono.ChBodyEasyCylinder(mysystem, 0.1, 0.5, 1000, True, True)
crankshaft.SetPos(chrono.ChVectorD(0, 0.5, 0))
crankshaft.SetRot(chrono.ChQuaternionD(chrono.ChVectorD(0, 1, 0), chrono.ChAngleD(chrono.CH_C_PI / 2)))
mysystem.Add(crankshaft)


connecting_rod = chrono.ChBodyEasyCylinder(mysystem, 0.05, 1.5, 1000, True, True)
connecting_rod.SetPos(chrono.ChVectorD(0, 1, 0))
mysystem.Add(connecting_rod)


piston = chrono.ChBodyEasyBox(mysystem, 0.2, 0.1, 0.2, 1000, True, True)
piston.SetPos(chrono.ChVectorD(1.5, 0, 0))
mysystem.Add(piston)


crankshaft_connecting_rod_joint = chrono.ChLinkRevolute()
crankshaft_connecting_rod_joint.Initialize(crankshaft, connecting_rod, chrono.ChFrameD(chrono.ChVectorD(0, 0.5, 0)))
mysystem.Add(crankshaft_connecting_rod_joint)

connecting_rod_piston_joint = chrono.ChLinkRevolute()
connecting_rod_piston_joint.Initialize(connecting_rod, piston, chrono.ChFrameD(chrono.ChVectorD(1.5, 0, 0)))
mysystem.Add(connecting_rod_piston_joint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crankshaft, floor, chrono.ChFrameD(chrono.ChVectorD(0, 0.5, 0)))
motor.SetSpeed(chrono.ChAngleD(chrono.CH_C_PI / 2))
mysystem.Add(motor)


mysystem.SetSolverType(chrono.ChSolver.Type_PSSOR)
mysystem.SetSolverMaxIterations(100)
mysystem.SetSolverTolerance(1e-6)
mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vis.AddSkyBox()
vis.AddLogo(0.5, 0.5, 0.01, 0.01)
vis.AddCameraVertical(chrono.ChVectorD(0, 1, 0))
vis.AddCameraUp(chrono.ChVectorD(0, 0, 1))
vis.AddTypicalLenght(1.0)


while vis.Run():
    mysystem.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()