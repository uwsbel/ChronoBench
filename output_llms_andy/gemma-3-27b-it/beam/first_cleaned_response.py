import pychrono as chrono
import pychrono.irrlicht as chronoirr






system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  






mesh = chrono.ChMesh()


mesh.AddNode(chrono.ChVectorD(0, 0, 0))  
mesh.AddNode(chrono.ChVectorD(1, 0, 0))  
mesh.AddNode(chrono.ChVectorD(2, 0, 0))  
mesh.AddNode(chrono.ChVectorD(0, 1, 0))  
mesh.AddNode(chrono.ChVectorD(1, 1, 0))  
mesh.AddNode(chrono.ChVectorD(2, 1, 0))  


mesh.AddBeam(0, 1, 0.1, chrono.ChVectorD(1, 0, 0))  
mesh.AddBeam(1, 2, 0.1, chrono.ChVectorD(1, 0, 0))  
mesh.AddBeam(0, 3, 0.1, chrono.ChVectorD(0, 1, 0))  
mesh.AddBeam(3, 4, 0.1, chrono.ChVectorD(1, 0, 0))  
mesh.AddBeam(4, 5, 0.1, chrono.ChVectorD(1, 0, 0))  
mesh.AddBeam(3, 0, 0.1, chrono.ChVectorD(0, -1, 0)) 
mesh.AddBeam(4, 1, 0.1, chrono.ChVectorD(0, -1, 0)) 
mesh.AddBeam(5, 2, 0.1, chrono.ChVectorD(0, -1, 0)) 




body = chrono.ChBodyEasy(mesh)
body.SetMass(1)
body.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(body)


body.SetFixed(True)






vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('PyChrono Beam Example')
vis.Initialize()
vis.AddTypicalCamera(chrono.ChVectorD(2, 1.5, -3))
vis.AddTypicalLights()





system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT)
system.SetStep(0.01)

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics()