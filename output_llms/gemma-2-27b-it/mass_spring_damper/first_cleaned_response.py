import pychrono as chrono
import pychrono.irrlicht as chronoirr






mysystem = chrono.ChSystemNSC()


mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))






vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(mysystem)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Mass-Spring-Damper System')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 5, -10))  
vis.AddLight(chrono.ChVectorD(5, 5, 5), chrono.ChColor(1, 1, 1))  







ground = chrono.ChBodyEasyBox(10, 0.2, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
mysystem.AddBody(ground)


mass = chrono.ChBodyEasySphere(0.5, 1000)
mass.SetPos(chrono.ChVectorD(0, 2, 0))
mysystem.AddBody(mass)


spring = chrono.ChLinkTSDA()
spring.Initialize(ground, mass, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0))
mysystem.AddLink(spring)







mass_shape = chrono.ChSphereShape()
mass_shape.GetSphereGeometry().rad = 0.5
mass.AddVisualShape(mass_shape)


spring_shape = chrono.ChCylinderShape()
spring_shape.GetCylinderGeometry().rad = 0.1
spring_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, 0)
spring_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, -2, 0)
spring.AddVisualShape(spring_shape)






while vis.Run():
    mysystem.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()