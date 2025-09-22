import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess


sys = chrono.ChSystemNSC()


floor = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, True)
floor.SetPos(chrono.ChVectorD(0, -5, 0))
sys.Add(floor)


crankshaft = chrono.ChBodyEasyCylinder(sys, 1, 0.5, 1000, True, True)
crankshaft.SetPos(chrono.ChVectorD(0, 0, 0))
sys.Add(crankshaft)


connecting_rod = chrono.ChBodyEasyBox(sys, 5, 0.5, 0.5, 1000, True, True)
connecting_rod.SetPos(chrono.ChVectorD(0, 2, 0))
sys.Add(connecting_rod)


piston = chrono.ChBodyEasyBox(sys, 1, 1, 1, 1000, True, True)
piston.SetPos(chrono.ChVectorD(0, 4, 0))
sys.Add(piston)


crankshaft_connecting_rod_revolute_joint = chrono.ChLinkRevolute()
crankshaft_connecting_rod_revolute_joint.Initialize(crankshaft, connecting_rod, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0), chrono.ChVectorD(0, 0, 1))
sys.Add(crankshaft_connecting_rod_revolute_joint)

connecting_rod_piston_spherical_joint = chrono.ChLinkSpherical()
connecting_rod_piston_spherical_joint.Initialize(connecting_rod, piston, chrono.ChVectorD(0, 2, 0), chrono.ChVectorD(0, 0, 0))
sys.Add(connecting_rod_piston_spherical_joint)


motor = chrono.ChLinkMotorRotation()
motor.Initialize(crankshaft, floor, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1))
motor.SetMotorFunction(chrono.ChFunction_Const(10))  
sys.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 0, 10))
vis.AddCamera(chrono.ChVectorD(0, 10, 0))
vis.AddTypicalLogo()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Crank-Slider Mechanism Simulation")


time = 0
dt = 0.01
while time < 10:
    sys.DoStepDynamics(dt)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    time += dt


sys.GetContactContainer().ReportAllContacts()