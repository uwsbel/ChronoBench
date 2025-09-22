import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath("..")  
system = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(10, 10, 1, 1000)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(ground)


mass = chrono.ChBodyEasySphere(0.5, 10)
mass.SetPos(chrono.ChVectorD(0, 1, 0))
system.Add(mass)


link = chrono.ChLinkTSDA(mass, ground)
link.SetSpringStiffness(1000)  
link.SetDampingCoefficient(100)  


mass_shape = chrono.ChShapeSphere(0.5)
mass.AddShape(mass_shape)

spring_shape = chrono.ChShapeCylinder(0.01, 1)
spring_shape.SetColor(chrono.ChColor(1, 0, 0))  
link.AddShape(spring_shape)


vis = chronoirr.ChIrrApp(system, 'Mass-Spring-Damper')
vis.AddCamera(chrono.ChVectorD(2, 2, 2), chrono.ChVectorD(0, 0, 0))
vis.SetAmbientLight(chrono.ChColor(0.1, 0.1, 0.1))
vis.SetSunLight(chrono.ChVectorD(1, 1, 1), chrono.ChColor(1, 1, 1))
vis.AddSky()
vis.AssetBindAll()


vis.SetTimestep(1/240)
vis.Start()