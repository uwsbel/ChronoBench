import pychrono.core as chrono
import pychrono.irrlicht as chronoirr



system = chrono.ChSystemNSC()


application = chronoirr.ChIrrApp(system, "Mass-Spring-Damper Simulation", chronoirr.dimension2du(800, 600))
application.Set window_title("Mass-Spring-Damper Demo")
application.Initialize()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(0, 0, 1.5), chronoirr.vector3df(0, 0, 0))
application.AssetBindAll()
application.AssetUpdateAll()


ground = chrono.ChBodyEasyBox(0.2, 0.02, 0.2, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetBodyFixed(True)
system.Add(ground)


mass_body = chrono.ChBodyEasyBox(0.2, 0.2, 0.2, 1000, True, True)
mass_body.SetPos(chrono.ChVectorD(0, 0.2, 0))
system.Add(mass_body)


spring = chrono.ChLinkTSDA()
spring.Initialize(ground, mass_body, False,
                  chrono.ChVectorD(0, 0.1, 0),  
                  chrono.ChVectorD(0, 0.1, 0))  


spring.SetSpringCoefficient(50.0)   
spring.SetDamperCoefficient(2.0)    
system.Add(spring)



visual_mass = chrono.ChBoxShape()
visual_mass.GetBoxGeometry().Size = chrono.ChVectorD(0.2, 0.2, 0.2)
mass_body.GetVisualShape(0).AddShape(visual_mass)




spring.GetVisualShape().SetColor(chrono.ChColor(0.8, 0.2, 0.2))
spring.GetVisualShape().SetShape(chrono.ChLineShape())


time_step = 1e-3
end_time = 2.0

while application.GetDevice().run():
    system.DoStepDynamics(time_step)
    application.AssetUpdateAll()
    application.BeginScene()
    application.DrawAll()
    application.EndScene()

    if system.GetChTime() > end_time:
        break