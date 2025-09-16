import pychrono as chrono
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(10, 1, 1, 1000, True, True)
system.Add(ground)


mass = chrono.ChBodyEasySphere(1, 1, 1000)
mass.SetPos(chrono.ChVectorD(0, 5, 0))
system.Add(mass)


spring_damper = chrono.ChLinkTSDA()
spring_damper.Initialize(mass, ground, chrono.ChCoordsysD(chrono.ChVectorD(0, 5, 0)))
system.Add(spring_damper)


spring_damper.SetSpringCoefficient(200)
spring_damper.SetDampingCoefficient(10)


app = irr.ChIrrApp(system, "Mass-Spring-Damper System", irr.dimension2du(800, 600))
app.AddTypicalLights()
app.AddCamera(irr.vector3df(0, 10, -20), irr.vector3df(0, 0, 0))
app.AddSkyBox()
app.AddLogo()
app.AddLightWithShadow(irr.vector3df(10, 10, -10), irr.vector3df(0, 0, 0), 20, 2, 10)


ground_shape = irr.ChVisualization.ChVisualization.GetDefaultSceneManager().AddBox(chrono.ChVectorD(10, 1, 1), 1000)
ground_shape.SetMaterial(irr.video.SColor(255, 200, 200, 200))
ground_shape.SetBody(ground)

mass_shape = irr.ChVisualization.ChVisualization.GetDefaultSceneManager().AddSphere(1, 1000)
mass_shape.SetMaterial(irr.video.SColor(255, 0, 0, 255))
mass_shape.SetBody(mass)

spring_shape = irr.ChVisualization.ChVisualization.GetDefaultSceneManager().AddCylinder(0.1, 5, 1000)
spring_shape.SetMaterial(irr.video.SColor(255, 0, 255, 0))
spring_shape.SetPos(chrono.ChVectorD(0, 2.5, 0))
spring_shape.SetRot(chrono.ChQuaternionD(1, 0, 0, chrono.CH_C_PI_2))


app.SetTimestep(0.01)
app.AssetBindAll()
app.AssetUpdateAll()
app.Run()