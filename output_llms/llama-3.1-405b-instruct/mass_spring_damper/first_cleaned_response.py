import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath('/path/to/chrono/data')


sys = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(sys,  
                              10, 10, 1,  
                              1000,  
                              True,  
                              True)  
ground.SetPos(chrono.ChVectorD(0, -5, 0))  
sys.Add(ground)  


mass = chrono.ChBodyEasySphere(sys,  
                               0.5,  
                               1000,  
                               False,  
                               True)  
mass.SetPos(chrono.ChVectorD(0, 5, 0))  
sys.Add(mass)  


spring = chrono.ChLinkTSDA()
spring.Initialize(mass, ground, False, chrono.ChVectorD(0, 5, 0), chrono.ChVectorD(0, -5, 0))
spring.SetSpringCoefficient(100)  
spring.SetDampingCoefficient(10)  
sys.Add(spring)  


app = chronoirr.ChIrrApp(sys, 'Mass-Spring-Damper System', chronoirr.dimension2du(800, 600))


app.AddTypicalLights()
app.AddCamera(chrono.ChVectorD(0, 0, -10))
app.AddSkyBox()


mass_shape = chrono.ChSphereShape()
mass_shape.GetSphereGeometry().rad = 0.5
mass.AddAsset(mass_shape)

spring_shape = chrono.ChCylinderShape()
spring_shape.GetCylinderGeometry().rad = 0.1
spring_shape.GetCylinderGeometry().height = 10
spring.AddAsset(spring_shape)


app.SetTimestep(0.01)
app.SetTryRealtime(True)

while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    sys.DoStepDynamics(app.GetTimestep())