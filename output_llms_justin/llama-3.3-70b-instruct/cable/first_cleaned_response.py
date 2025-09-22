import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


sys = chrono.ChSystemNSC()


sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, chrono.ChVectorD(0, -5, 0))
sys.Add(ground)


beam = chrono.ChBodyEasyCableANCF(sys, 10, 0.1, 1000, True, chrono.ChVectorD(0, 5, 0))
sys.Add(beam)


beam.GetMaterial().SetYoungModulus(2e11)
beam.GetMaterial().SetPoissonsRatio(0.3)
beam.GetMaterial().SetDampingRatio(0.1)


hinge = chrono.ChLinkMateFixed()
hinge.SetName("hinge")
hinge.Initialize(beam, ground, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(0)))
sys.Add(hinge)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 10, -20))
vis.AddTypicalLights()


time_step = 0.01
end_time = 10
while sys.GetChTime() < end_time:
    sys.DoStepDynamics(time_step)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    vis.Run()

    
    print("Time: ", sys.GetChTime())
    for i in range(beam.GetNDofN()):
        print("Node {}: {}".format(i, beam.GetNodeN(i).GetPos()))


vis.CloseWindow()