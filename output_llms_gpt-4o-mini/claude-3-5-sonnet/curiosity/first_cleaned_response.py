import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath("../../../data/")


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(20, 1, 20, 1000, True)
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
system.Add(ground)


box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True)
box.SetPos(chrono.ChVectorD(0, 5, 0))
system.Add(box)


sphere = chrono.ChBodyEasySphere(0.5, 1000, True)
sphere.SetPos(chrono.ChVectorD(2, 5, 0))
system.Add(sphere)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("PyChrono example")
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 14, -20))
vis.AddTypicalLights()


time_step = 0.01
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)

vis.GetDevice().closeDevice()