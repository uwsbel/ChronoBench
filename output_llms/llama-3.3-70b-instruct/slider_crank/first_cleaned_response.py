import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.engine as chronoengine


sys = chrono.ChSystemNSC()


floor = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, True)
floor.SetPos(chrono.ChVectorD(0, -5, 0))
floor.SetMaterial(chrono.ChMaterialSurfaceNSC())


crankshaft = chrono.ChBodyEasyCylinder(sys, 1, 0.1, 1000, True, True)
crankshaft.SetPos(chrono.ChVectorD(0, 0, 0))
crankshaft.SetMaterial(chrono.ChMaterialSurfaceNSC())


connecting_rod = chrono.ChBodyEasyBox(sys, 2, 0.1, 0.1, 1000, True, True)
connecting_rod.SetPos(chrono.ChVectorD(2, 0, 0))
connecting_rod.SetMaterial(chrono.ChMaterialSurfaceNSC())


piston = chrono.ChBodyEasyBox(sys, 1, 1, 1, 1000, True, True)
piston.SetPos(chrono.ChVectorD(4, 0, 0))
piston.SetMaterial(chrono.ChMaterialSurfaceNSC())


sys.Add(floor)
sys.Add(crankshaft)
sys.Add(connecting_rod)
sys.Add(piston)


joint1 = chrono.ChLinkRevolute()
joint1.Initialize(crankshaft, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
joint1.AddChild(connecting_rod)
sys.Add(joint1)

joint2 = chrono.ChLinkRevolute()
joint2.Initialize(connecting_rod, chrono.ChFrameD(chrono.ChVectorD(2, 0, 0)))
joint2.AddChild(piston)
sys.Add(joint2)

joint3 = chrono.ChLinkRevolute()
joint3.Initialize(piston, chrono.ChFrameD(chrono.ChVectorD(4, 0, 0)))
joint3.AddChild(floor)
sys.Add(joint3)


motor = chrono.ChLinkMotorRotation()
motor.Initialize(crankshaft, floor, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
motor.SetMotorFunction(chrono.ChFunction_Const(chrono.ChVectorD(0, 0, 10)))
sys.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 10, -10))
vis.AddCamera(chrono.ChVectorD(0, 10, 10))
vis.AddTypicalLogo()
vis.SetLogoTexture(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.SetLogoGetAttention(True)
vis.SetLogoPosition(chrono.ChVectorD(10, 10, 0))


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()