import pychrono as chrono
import pychrono.irrlicht as chronoirr




system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank-Slider Mechanism')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.5, 0.5, 1), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()




material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.4)
material.SetRestitution(0.1)




floor = chrono.ChBodyEasyBox(2, 0.1, 2, 1000, True, True, material)
floor.SetBodyFixed(True)
system.Add(floor)




crankshaft = chrono.ChBodyEasyCylinder(0.02, 0.1, 1000, True, True, material)
crankshaft.SetPos(chrono.ChVectorD(0, 0.1, 0))
system.Add(crankshaft)

connecting_rod = chrono.ChBodyEasyBox(0.02, 0.25, 0.02, 1000, True, True, material)
connecting_rod.SetPos(chrono.ChVectorD(0.1, 0.1, 0))
system.Add(connecting_rod)

piston = chrono.ChBodyEasyCylinder(0.04, 0.1, 1000, True, True, material)
piston.SetPos(chrono.ChVectorD(0.2, 0.1, 0))
system.Add(piston)




crank_joint = chrono.ChLinkRevolute(system, crankshaft, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
system.AddLink(crank_joint)

rod_crank_joint = chrono.ChLinkSpherical(system, connecting_rod, crankshaft, chrono.ChCoordsysD(chrono.ChVectorD(0.1, 0, 0)))
system.AddLink(rod_crank_joint)

rod_piston_joint = chrono.ChLinkLockRevolute(system, connecting_rod, piston, chrono.ChCoordsysD(chrono.ChVectorD(0.2, 0, 0), chrono.Q_from_AngY(chrono.CH_C_PI_2)))
system.AddLink(rod_piston_joint)




motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crankshaft, crank_joint.GetFrame2(), chrono.ChFrameMovingD())
motor.SetSpeedFunction(chrono.ChFunction_Const(chrono.CH_C_PI))  
system.AddLink(motor)




while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(1e-3)