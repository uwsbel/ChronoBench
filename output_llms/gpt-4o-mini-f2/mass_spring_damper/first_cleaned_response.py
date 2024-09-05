import pychrono as chrono
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  


ground = chrono.ChBody()
system.AddBody(ground)
ground.SetFixed(True)  
ground.EnableCollision(False)  


box = chrono.ChVisualShapeBox(4, 0.5, 0.5)
ground.AddVisualShape(box, chrono.ChFramed(chrono.ChVector3d(-2, 0, 0)))


mass = chrono.ChBody()
system.AddBody(mass)
mass.SetFixed(False)  
mass.EnableCollision(False)  
mass.SetMass(1)  
mass.SetInertiaXX(chrono.ChVector3d(1, 1, 1))  


cylinder = chrono.ChVisualShapeCylinder(0.5, 1.5)
mass.AddVisualShape(cylinder, chrono.ChFramed(chrono.ChVector3d(2, 0, 0), chrono.QuatFromAngleY(chrono.CH_PI_2)))


spring = chrono.ChLinkTSDA()
system.AddLink(spring)
spring.Initialize(ground, mass, True,  
                  chrono.ChVector3d(-2, 0, 0),  
                  chrono.ChVector3d(2, 0, 0))  


spring.SetRestLength(1)  
spring.SetKspring(10)    
spring.SetDdamp(1)       


spring.AddVisualShape(chrono.ChVisualShapeSpring(0.1, 20, 40))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Mass-spring-damper system')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 6.5), chrono.ChVector3d(0, 0, 0), 3, 4, 10, 40, 512)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(1e-3)