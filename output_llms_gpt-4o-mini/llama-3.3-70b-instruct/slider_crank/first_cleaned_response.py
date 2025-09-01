import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.engine as chronoengine


sys = chrono.ChSystemNSC()


floor = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, True)
floor.SetPos(chrono.ChVectorD(0, -5, 0))
sys.Add(floor)


crankshaft = chrono.ChBodyEasyCylinder(sys, 0.5, 2, 1000, True, True)
crankshaft.SetPos(chrono.ChVectorD(0, 0, 0))
sys.Add(crankshaft)


connecting_rod = chrono.ChBodyEasyBox(sys, 2, 0.5, 0.5, 1000, True, True)
connecting_rod.SetPos(chrono.ChVectorD(2, 0, 0))
sys.Add(connecting_rod)


piston = chrono.ChBodyEasyBox(sys, 1, 1, 1, 1000, True, True)
piston.SetPos(chrono.ChVectorD(4, 0, 0))
sys.Add(piston)


crankshaft_connecting_rod_joint = chrono.ChLinkRevolute()
crankshaft_connecting_rod_joint.Initialize(crankshaft, connecting_rod, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(2, 0, 0), chrono.ChVectorD(0, 0, 1))
sys.Add(crankshaft_connecting_rod_joint)

connecting_rod_piston_joint = chrono.ChLinkRevolute()
connecting_rod_piston_joint.Initialize(connecting_rod, piston, chrono.ChVectorD(2, 0, 0), chrono.ChVectorD(4, 0, 0), chrono.ChVectorD(0, 0, 1))
sys.Add(connecting_rod_piston_joint)


motor = chrono.ChLinkMotorRotation()
motor.Initialize(crankshaft, sys.GetGroundBody(), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1))
motor.SetMaxTorque(1000)
motor.SetMotorFunction(chrono.ChFunction_Const(1))
sys.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 0, 10))
vis.AddCamera(chrono.ChVectorD(0, 10, 0))
vis.AddCamera(chrono.ChVectorD(10, 0, 0))
vis.AddTypicalLogo()
vis.SetLogoTexture(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.SetLogoGetAttention(True)
vis.SetLogoPosition(chrono.ChVectorD(10, 10, 0))
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Crank-Slider Mechanism Simulation")


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()