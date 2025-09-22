import pychrono as chrono
import pychrono.irrlicht as irr


my_system = chrono.ChSystemNSC()


my_application = irr.ChIrrApp(my_system, 'Mass-Spring-Damper Demo', irr.dimension2du(800, 600))
my_application.AddTypicalLogo()
my_application.AddTypicalSky()
my_application.AddTypicalLights()
my_application.SetCameraPosition(chrono.ChVectorD(3, 3, 3))
my_application.SetCameraTarget(chrono.ChVectorD(1, 0, 0))


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
my_system.Add(ground)


ground_shape = chrono.ChBoxShape()
ground_shape.GetBoxGeometry().SetLengths(chrono.ChVectorD(0.1, 0.1, 0.1))
ground_shape.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
ground.AddAsset(ground_shape)


mass = chrono.ChBody()
mass.SetMass(1)  
mass.SetPos(chrono.ChVectorD(1, 0, 0))  
my_system.Add(mass)


mass_shape = chrono.ChBoxShape()
mass_shape.GetBoxGeometry().SetLengths(chrono.ChVectorD(0.2, 0.2, 0.2))
mass_shape.SetColor(chrono.ChColor(0.8, 0.3, 0.3))
mass.AddAsset(mass_shape)


springdamper = chrono.ChLinkTSDA()
springdamper.Initialize(
    ground, mass,  
    chrono.ChFrameD(),  
    chrono.ChFrameD()   
)
springdamper.SetSpringK(100)      
springdamper.SetSpringRestLength(0)  
springdamper.SetDampingR(10)      
my_system.AddLink(springdamper)


spring_shape = irr.ChSpringShape(springdamper, 10, 0.1)  
springdamper.AddAsset(spring_shape)


my_application.AssetBind()
my_application.AssetUpdate()
my_application.SetTimestep(0.005)  


my_application.Run()