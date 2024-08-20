import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath("../data/")


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(20, 1, 20, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
ground.SetBodyFixed(True)
system.Add(ground)


sphere = chrono.ChBodyEasySphere(1, 1000, True, True)
sphere.SetPos(chrono.ChVectorD(0, 10, 0))
sphere.SetPos_dt(chrono.ChVectorD(5, 0, 0))  
system.Add(sphere)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Basic PyChrono Simulation")
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 14, -20))
vis.AddTypicalLights()


time_step = 0.01
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)

    
    if system.GetChTime() > 10:
        break

print("Simulation complete")