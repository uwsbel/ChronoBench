import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.engine as chronoengine


sys = chrono.ChSystemNSC()


sys.DefaultWarnings = False


floor = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, True)
floor.SetPos(chrono.ChVectorD(0, -5, 0))
floor.SetMaterial(chrono.ChMaterialSurfaceNSC())


crankshaft = chrono.ChBodyEasyCylinder(sys, 1, 0.5, 1000, True, True)
crankshaft.SetPos(chrono.ChVectorD(0, 0, 0))
crankshaft.SetMaterial(chrono.ChMaterialSurfaceNSC())


connecting_rod = chrono.ChBodyEasyBox(sys, 5, 0.5, 0.5, 1000, True, True)
connecting_rod.SetPos(chrono.ChVectorD(2, 0, 0))
connecting_rod.SetMaterial(chrono.ChMaterialSurfaceNSC())


piston = chrono.ChBodyEasyBox(sys, 1, 1, 1, 1000, True, True)
piston.SetPos(chrono.ChVectorD(5, 0, 0))
piston.SetMaterial(chrono.ChMaterialSurfaceNSC())


sys.Add(floor)
sys.Add(crankshaft)
sys.Add(connecting_rod)
sys.Add(piston)


joint1 = chrono.ChLinkRevolute()
joint1.Initialize(crankshaft, connecting_rod, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(2, 0, 0), chrono.ChVectorD(0, 0, 1))
sys.Add(joint1)

joint2 = chrono.ChLinkRevolute()
joint2.Initialize(connecting_rod, piston, chrono.ChVectorD(5, 0, 0), chrono.ChVectorD(2, 0, 0), chrono.ChVectorD(0, 0, 1))
sys.Add(joint2)


motor = chrono.ChLinkMotorRotation()
motor.Initialize(crankshaft, floor, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1))
motor.SetSpeed(10)  
sys.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 0, 10))
vis.AddTypicalLogo()
vis.SetWindowSize(1024, 768)
vis.SetCameraAZEL(chrono.ChVectorD(0, 0, 10), 45, 45, 0)


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()