import pychrono as chrono
import pychrono.irrlicht as chronoirr


rest_length = 1.5
spring_coef = 50
damping_coef = 1


sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, 0))  

ground = chrono.ChBodyEasyBox(chrono.ChVectorD(10, 1, 10), 1000)  
sys.Add(ground)  
ground.SetFixed(True)
ground.SetCollide(False)  

sph_1 = chrono.ChVisualShapeSphere(chrono.ChVectorD(0.1, 0.1, 0.1))  
ground.AddVisualShape(sph_1)

body_1 = chrono.ChBodyEasyBox(chrono.ChVectorD(1, 1, 1), 1000)  
sys.Add(body_1)  
body_1.SetPos(chrono.ChVectorD(-1, -3, 0))  
body_1.SetFixed(False)
body_1.SetCollide(False)  

box_1 = chrono.ChVisualShapeBox(chrono.ChVectorD(1, 1, 1))  
box_1.SetColor(chrono.ChColor(0.6, 0, 0))
body_1.AddVisualShape(box_1)



spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground, True, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(-1, 0, 0))  
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
sys.Add(spring_1)  


spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))


body_2 = chrono.ChBodyEasyBox(chrono.ChVectorD(1, 1, 1), 1000)
sys.Add(body_2)
body_2.SetPos(chrono.ChVectorD(1, -3, 0))
body_2.SetFixed(False)
body_2.SetCollide(False)

body_3 = chrono.ChBodyEasyBox(chrono.ChVectorD(1, 1, 1), 1000)
sys.Add(body_3)
body_3.SetPos(chrono.ChVectorD(3, -3, 0))
body_3.SetFixed(False)
body_3.SetCollide(False)


spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_1, body_2, True, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(1, 0, 0))
spring_2.SetRestLength(rest_length)
spring_2.SetSpringCoefficient(spring_coef)
spring_2.SetDampingCoefficient(damping_coef)
sys.Add(spring_2)

spring_3 = chrono.ChLinkTSDA()
spring_3.Initialize(body_2, body_3, True, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(1, 0, 0))
spring_3.SetRestLength(rest_length)
spring_3.SetSpringCoefficient(spring_coef)
spring_3.SetDampingCoefficient(damping_coef)
sys.Add(spring_3)

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ChLinkTSDA demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, 6))  
vis.AddTypicalLights()

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)