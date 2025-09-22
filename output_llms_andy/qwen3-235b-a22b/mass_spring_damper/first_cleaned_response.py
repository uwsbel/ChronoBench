import pychrono as chrono
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.SetGravity(chrono.ChVectorD(0, -9.81, 0))  


mass_body = chrono.ChBody()
mass_body.SetMass(1.0)
mass_body.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))  
mass_body.SetPos(chrono.ChVectorD(0, 1, 0))  
mass_body.SetBodyFixed(False)
system.Add(mass_body)


box_shape = chrono.ChBoxShape()
box_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.2, 0.2, 0.2)  
mass_body.AddAsset(box_shape)


color = chrono.ChColorAsset()
color.SetColor(chrono.ChColor(0.6, 0.6, 0.6))  
mass_body.AddAsset(color)


link = chrono.ChLinkTSDA()


ground_point = chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0))
mass_point = chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0))


link.Initialize(system.GetGroundBody(), mass_body, False, ground_point, mass_point)
system.Add(link)


link.SetSpringK(100)  
link.SetDampingR(5)   


spring_shape = chrono.ChSpringShape(0.05, 10, 0.4)  
link.AddAsset(spring_shape)


application = chronoirr.ChIrrApp(system, "Mass-Spring-Damper Simulation", chronoirr.dimension2du(1024, 768))
application.AddTypicalLights()
application.AddCamera(chronoirr.vector3df(0, 3, -6), chronoirr.vector3df(0, 1, 0))  


application.CameraUseAutoRotate(True)


application.SetTimestep(0.01)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    system.DoStepDynamics(application.GetTimestep())