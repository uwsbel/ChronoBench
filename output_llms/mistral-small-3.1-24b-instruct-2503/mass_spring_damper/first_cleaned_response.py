import pychrono as chrono
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


ground = chrono.ChBodyEasyBox(1, 1, 0.1, 1000)
ground.SetPos(chrono.ChVectorD(0, -0.5, 0))
ground.SetBodyFixed(True)
system.Add(ground)


mass = chrono.ChBodyEasySphere(0.1, 1, True, True)
mass.SetPos(chrono.ChVectorD(0, 0.5, 0))
mass.SetMass(1)
system.Add(mass)


spring_damper = chrono.ChLinkTSDA()
spring_damper.Initialize(ground, mass, False, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0.5, 0), False)
spring_damper.Set_SpringCoefficient(1000)  
spring_damper.Set_DampingCoefficient(10)   
system.Add(spring_damper)


application = chronoirr.ChIrrApp(system, 'Mass-Spring-Damper System', chrono.ChVectorD(0.2, 0.2, 0.2))
application.AddTypicalSky()
application.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_small.png'))
application.AddTypicalCamera(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 0, 0))
application.AddTypicalLights()


mass_shape = chronoirr.ChIrrNodePointCloud()
mass_shape.AttachBody(mass)
application.GetSceneManager().getRootSceneNode().addChild(mass_shape)


spring_shape = chronoirr.ChIrrNodeLine()
spring_shape.ClearPoints()
spring_shape.PushPoint(chrono.ChVectorD(0, 0, 0))
spring_shape.PushPoint(chrono.ChVectorD(0, 0.5, 0))
spring_shape.SetDiameter(0.01)
spring_shape.SetColorf(chrono.ChColorf(1, 0, 0))  
application.GetSceneManager().getRootSceneNode().addChild(spring_shape)


application.SetTimestep(0.01)
application.SetTryRealTime(True)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    system.DoStepDynamics(application.GetTimestep())
    application.EndScene()