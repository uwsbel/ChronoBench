import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.postprocess as postprocess


system = chrono.ChSystemNSC()


collision_model = chrono.ChCollisionModelParallel()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


floor_body = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True)
floor_body.SetPos(chrono.ChVectorD(0, -1, 0))
floor_body.SetBodyFixed(True)
system.Add(floor_body)


crankshaft = chrono.ChBodyEasyCylinder(0.1, 0.5, 1000, True, True)
crankshaft.SetPos(chrono.ChVectorD(0, 0, 0))
crankshaft.SetRot(chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD(1, 0, 0)))
system.Add(crankshaft)


rod = chrono.ChBodyEasyBox(0.1, 1, 0.1, 1000, True, True)
rod.SetPos(chrono.ChVectorD(0.5, 0, 0))
system.Add(rod)


piston = chrono.ChBodyEasyCylinder(0.2, 0.5, 1000, True, True)
piston.SetPos(chrono.ChVectorD(1, 0, 0))
system.Add(piston)


crank_joint = chrono.ChLinkRevolute()
crank_joint.Initialize(crankshaft, floor_body, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
system.Add(crank_joint)


crank_motor = chrono.ChLinkMotorRotationSpeed()
crank_motor.Initialize(crankshaft, floor_body, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
crank_motor.SetMotorFunction(chrono.ChFunction_Const(1))  
system.Add(crank_motor)


crank_rod_joint = chrono.ChLinkRevolute()
crank_rod_joint.Initialize(crankshaft, rod, chrono.ChCoordsysD(chrono.ChVectorD(0.5, 0, 0)))
system.Add(crank_rod_joint)


rod_piston_joint = chrono.ChLinkRevolute()
rod_piston_joint.Initialize(rod, piston, chrono.ChCoordsysD(chrono.ChVectorD(1, 0, 0)))
system.Add(rod_piston_joint)


piston_joint = chrono.ChLinkLockPrismatic()
piston_joint.Initialize(piston, floor_body, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD(0, 1, 0))))
system.Add(piston_joint)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank-Slider Mechanism')
vis.EnableShadows()
vis.AddCamera(chrono.ChVectorD(2.5, 2.5, 2.5), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()


vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddLightWithShadow(chrono.ChVectorD(2, 5, 2), chrono.ChVectorD(0, -1, 0), 5, 0.1, 10)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    system.DoStepDynamics(0.01)
    vis.EndScene()


vis.GetDevice().closeDevice()