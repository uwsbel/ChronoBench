import pychrono as chrono
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, 0))  


ground = system.GetGroundBody()


mass = chrono.ChBody()
mass.SetName("Mass")
mass.SetMass(1.0)
mass.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
mass.SetPos(chrono.ChVectorD(1.5, 0, 0))  
system.AddBody(mass)
mass.SetBodyFixed(False)


spring = chrono.ChLinkTSDA()
spring.Initialize(ground, mass, 
                  chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 
                  chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
spring.SetRestLength(1.0)
spring.SetSpringCoefficient(100.0)
spring.SetDampingCoefficient(10.0)
system.AddLink(spring)


mass_shape = chrono.ChVisualShapeSphere(0.2)
mass_shape.SetColor(chrono.ChColor(0.6, 0.2, 0.2))
mass.AddVisualShape(mass_shape, chrono.ChFrameD())


spring_shape = chrono.ChVisualShapeSpring(0.05, 80, 10)
spring_shape.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
spring.AddVisualShape(spring_shape, chrono.ChFrameD())


app = chronoirr.ChIrrApp(system, "Mass-Spring-Damper System", chronoirr.dimension2du(1024, 768))
app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalCamera(chronoirr.vector3df(2, 1, 1), chronoirr.vector3df(0.5, 0, 0))
app.AddLightWithShadow(chronoirr.vector3df(5, 5, 5), chronoirr.vector3df(0, 0, 0), 3, 2, 40, 50, 500)
app.SetShowInfos(True)
app.SetContactsDrawMode(4)
app.SetSymbolsDrawMode(2)


time_step = 0.001
app.SetTimestep(time_step)


while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    system.DoStepDynamics(time_step)
    app.EndScene()