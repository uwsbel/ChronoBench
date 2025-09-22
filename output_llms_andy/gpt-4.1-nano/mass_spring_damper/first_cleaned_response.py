import pychrono.core as chrono
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()


application = irr.ChIrrApp(system, "Mass-Spring-Damper System", irr.dimension2du(800, 600))
application.AddTypicalLights()
application.SetCameraPosition(chrono.ChVectorD(0, 1, 3))
application.SetChaseCamera(chrono.ChVectorD(0, 0, 0))
application.EnableStatistics(False)
application.DrawAllPreseed = True


ground = chrono.ChBodyEasyBox(1.0, 0.1, 1.0, 1000, True, True)
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, -0.05, 0))
system.Add(ground)


mass_radius = 0.1
mass_height = 0.2
mass_body = chrono.ChBodyEasyCylinder(mass_radius, mass_height, 1000, True, True)
mass_body.SetPos(chrono.ChVectorD(0, 0.5, 0))
system.Add(mass_body)





spring_stiffness = 1000  
damper_coefficient = 10  
rest_length = 0.5       


spring_link = chrono.ChLinkTSDA()
spring_link.SetId(0)
spring_link.Initialize(
    ground,          
    mass_body,       
    False,           
    chrono.ChVectorD(0, 0, 0),  
    chrono.ChVectorD(0, 0.5, 0) 
)


spring_link.SetSpringCoefficient(spring_stiffness)
spring_link.SetDampingCoefficient(damper_coefficient)
spring_link.SetRestLength(rest_length)

system.Add(spring_link)





application.AssetBindAll()
application.AssetUpdateAll()


while application.GetDevice().run():
    system.DoStepDynamics(0.01)
    application.AssetUpdateAll()
    application.BeginScene()
    application.DrawAll()
    application.EndScene()