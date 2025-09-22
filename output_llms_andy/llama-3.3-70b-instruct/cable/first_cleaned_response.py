import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as chrono_vehicle


sys = chrono.ChSystemNSC()


sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
ground.SetBodyFixed(True)


beam = chrono.ChBodyEasyCableANCF(sys, 10, 0.1, 1000, 10)


beam.SetPos(chrono.ChVectorD(0, 5, 0))
beam.SetRot(chrono.Q_from_AngX(0))


hinge = chrono.ChLinkMateGeneric()
hinge.Initialize(beam, ground, False, chrono.ChVectorD(0, 5, 0), chrono.ChVectorD(0, -5, 0))
sys.Add(hinge)


sys.Add(beam)
sys.Add(ground)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 10, -20))
vis.AddTypicalLights()


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()