import pychrono as chrono
import pychrono.irrlicht as chronoirr







system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachTo(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Crank-Slider Mechanism")
vis.Initialize()


floor = chrono.ChBodyEasyBox(2, 0.1, 2, 1000, True, True)
floor.SetPos(chrono.ChVectorD(0, -0.5, 0))
system.Add(floor)


crankshaft = chrono.ChBodyEasyCylinder(0.1, 0.3, 1000, True, True)
crankshaft.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(crankshaft)


connecting_rod = chrono.ChBodyEasyBox(0.2, 0.05, 0.8, 1000, True, True)
connecting_rod.SetPos(chrono.ChVectorD(0.3, 0, 0))
system.Add(connecting_rod)


piston = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000, True, True)
piston.SetPos(chrono.ChVectorD(0.3 + 0.8, 0, 0))
system.Add(piston)


rev_joint_crank = chrono.ChLinkRevolute()
rev_joint_crank.Initialize(floor, crankshaft, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1))
system.Add(rev_joint_crank)

rev_joint_rod = chrono.ChLinkRevolute()
rev_joint_rod.Initialize(crankshaft, connecting_rod, chrono.ChVectorD(0.3, 0, 0), chrono.ChVectorD(0, 0, 1))
system.Add(rev_joint_rod)

rev_joint_piston = chrono.ChLinkRevolute()
rev_joint_piston.Initialize(connecting_rod, piston, chrono.ChVectorD(0.3 + 0.8, 0, 0), chrono.ChVectorD(0, 0, 1))
system.Add(rev_joint_piston)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(floor, crankshaft, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1))
motor.SetSpeedFunction(chrono.ChFunction_Const(chrono.CH_C_PI))  
system.Add(motor)








vis.GetCamera().SetLocation(chrono.ChVectorD(2, 2, 2))
vis.GetCamera().SetLookAt(chrono.ChVectorD(0, 0, 0))


vis.AddLight(chrono.ChLightDirectional(chrono.ChVectorD(0, -1, 0)))










while vis.Run():
    system.DoStepDynamics(0.01)
    vis.Render()