import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath("/path/to/chrono/data/")


sys = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(sys,  
                              1, 1, 1,  
                              1000,  
                              True,  
                              True,  
                              chrono.ChVectorD(0, -0.5, 0),  
                              chrono.ChQuaternionD(1, 0, 0, 0))  


mass = chrono.ChBodyEasySphere(sys,  
                               0.1,  
                               1000,  
                               True,  
                               True,  
                               chrono.ChVectorD(0, 1, 0),  
                               chrono.ChQuaternionD(1, 0, 0, 0))  


spring = chrono.ChLinkTSDA()
spring.Initialize(ground, mass, True, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0))
sys.Add(spring)


spring.SetSpringCoefficient(100)
spring.SetDampingCoefficient(10)


app = chronoirr.ChIrrApp(sys, "Mass-Spring-Damper Demo", chronoirr.dimension2du(800, 600))


app.AddCamera(chronoirr.vector3df(0, 0, -3))
app.AddTypicalLights()


mass_shape = chrono.ChSphereShape()
mass_shape.GetSphereGeometry().rad = 0.1
mass.AddAsset(mass_shape)

spring_shape = chrono.ChCylinderShape()
spring_shape.GetCylinderGeometry().rad = 0.01
spring_shape.GetCylinderGeometry().height = 1.5
spring.AddAsset(spring_shape)


app.SetTimestep(0.01)
app.SetTryRealtime(True)
while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    sys.DoStepDynamics(app.GetTimestep())