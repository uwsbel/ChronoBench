import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath('../data/')  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


ground = chrono.ChBodyEasyBox(1, 1, 0.1, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.5, 0))
ground.SetBodyFixed(True)
system.AddBody(ground)


mass = chrono.ChBodyEasySphere(0.1, 1, True, True)
mass.SetPos(chrono.ChVectorD(0, 0.5, 0))
system.AddBody(mass)


spring_damper = chrono.ChLinkTSDA()
spring_damper.Initialize(mass, ground, False, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0), False)
spring_damper.Set_Spring_K(1000)  
spring_damper.Set_Damping_R(10)  
spring_damper.Set_RestLength(0.5)  
system.AddLink(spring_damper)


mass_shape = chrono.ChSphereShape()
mass_shape.GetSphereGeometry().rad = 0.1
mass_shape.GetMaterialSurface().SetFriction(0.3)
mass_shape.GetMaterialSurface().SetRestitution(0.1)
mass_shape.GetMaterialSurface().SetCompliance(0.0)
mass_shape.GetMaterialSurface().SetComplianceT(0.0)
mass.AddVisualShape(mass_shape)

spring_shape = chrono.ChCylinderShape()
spring_shape.GetCylinderGeometry().rad = 0.01
spring_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, 0)
spring_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, -0.5, 0)
spring_shape.GetMaterialSurface().SetFriction(0.3)
spring_shape.GetMaterialSurface().SetRestitution(0.1)
spring_shape.GetMaterialSurface().SetCompliance(0.0)
spring_shape.GetMaterialSurface().SetComplianceT(0.0)
mass.AddVisualShape(spring_shape)


application = chronoirr.ChIrrApp(system, 'Mass-Spring-Damper System', chronoirr.dimension2du(800, 600))
application.AddTypicalSky()
application.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_chrono_alpha.png')
application.AddTypicalCamera(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 0, 0))
application.AddTypicalLights()
application.SetSymbolScale(0.05)


application.SetTimestep(0.01)
application.SetTryRealTime(True)


application.Run()