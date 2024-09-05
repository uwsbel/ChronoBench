import pychrono as chrono
import pychrono.irrlicht as vis


chrono.SetChronoDataPath("..")  
system = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(10, 10, 1, 1000)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(ground)


mass = chrono.ChBodyEasySphere(0.5, 1)
mass.SetPos(chrono.ChVectorD(0, 1, 0))
system.Add(mass)


link = chrono.ChLinkTSDA(mass, ground)
link.SetSpringStiffness(1000)  
link.SetDampingCoefficient(100)  


system.Add(link)


vis.Add(system)
vis.SetCamera(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 0, 0))
vis.SetAmbientLight(chrono.ChColor(0.5, 0.5, 0.5))
vis.SetDirectionalLight(chrono.ChColor(1, 1, 1), chrono.ChVectorD(1, 1, 1))


mass.GetVisualShape().SetMaterial(chrono.ChMaterialSurface(chrono.ChColor(1, 0, 0)))  
link.GetVisualShape().SetMaterial(chrono.ChMaterialSurface(chrono.ChColor(0, 1, 0)))  


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
vis.Run()