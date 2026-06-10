import pychrono as chrono
import pychrono.irrlicht as chronoirr




system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))




mass = 1.0               
spring_coef = 50.0       
damping_coef = 1.0       
rest_length = 1.5        




ground = chrono.ChBody()
ground.SetFixed(True)
ground.EnableCollision(False)
system.Add(ground)


ground_marker = chrono.ChVisualShapeSphere(0.05)
ground_marker.SetColor(chrono.ChColor(0.2, 0.2, 0.2))
ground.AddVisualShape(ground_marker, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))




body = chrono.ChBody()
body.SetMass(mass)
body.SetPos(chrono.ChVector3d(0, -rest_length, 0))   
body.EnableCollision(False)
system.Add(body)


box = chrono.ChVisualShapeBox(0.3, 0.3, 0.3)
box.SetColor(chrono.ChColor(0.6, 0.2, 0.2))
body.AddVisualShape(box, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))




spring = chrono.ChLinkTSDA()

spring.Initialize(
    body,                              
    ground,                            
    True,                              
    chrono.ChVector3d(0, -rest_length, 0),  
    chrono.ChVector3d(0, 0, 0)         
)


spring.SetRestLength(rest_length)
spring.SetSpringCoefficient(spring_coef)
spring.SetDampingCoefficient(damping_coef)
system.Add(spring)


spring.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Mass-Spring-Damper (ChLinkTSDA) Demo')
vis.Initialize()
vis.AddLogo(chronoirr.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -1, 4), chrono.ChVector3d(0, -1, 0))
vis.AddTypicalLights()
vis.AddLight(chrono.ChVector3d(2, 5, 3), 10, chrono.ChColor(0.8, 0.8, 0.8))




time_step = 1e-3

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)