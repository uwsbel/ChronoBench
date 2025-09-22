import pychrono as chrono
import pychrono.irrlicht as chronoirr


rest_length = 1.5
spring_coef = 50
damping_coef = 1


sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0)) 


ground = chrono.ChBody()
sys.AddBody(ground)
ground.SetFixed(True)
ground.EnableCollision(False)


sph_ground_marker = chrono.ChVisualShapeSphere(0.1)
sph_ground_marker.SetColor(chrono.ChColor(0.3, 0.3, 0.3)) 
ground.AddVisualShape(sph_ground_marker, chrono.ChFramed(chrono.ChVector3d(-1, 0, 0)))


body_1 = chrono.ChBody()
sys.AddBody(body_1)
body_1.SetPos(chrono.ChVector3d(-1, -3, 0)) 
body_1.SetFixed(False)
body_1.EnableCollision(False)
body_1.SetMass(1.0)
body_1.SetInertiaXX(chrono.ChVector3d(1, 1, 1)) 


box_1_shape = chrono.ChVisualShapeBox(1, 1, 1)
box_1_shape.SetColor(chrono.ChColor(0.6, 0, 0)) 
body_1.AddVisualShape(box_1_shape)


spring_1 = chrono.ChLinkTSDA()


spring_1.Initialize(body_1, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(-1, 0, 0))
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_1)


spring_1_shape = chrono.ChVisualShapeSpring(0.05, 80, 15)
spring_1.AddVisualShape(spring_1_shape)




body_2 = chrono.ChBody()
sys.AddBody(body_2)
body_2.SetPos(chrono.ChVector3d(-1, -6, 0)) 
body_2.SetFixed(False)
body_2.EnableCollision(False)
body_2.SetMass(1.0)
body_2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))


box_2_shape = chrono.ChVisualShapeBox(1, 1, 1)
box_2_shape.SetColor(chrono.ChColor(0, 0.6, 0)) 
body_2.AddVisualShape(box_2_shape)


body_3 = chrono.ChBody()
sys.AddBody(body_3)
body_3.SetPos(chrono.ChVector3d(-1, -9, 0)) 
body_3.SetFixed(False)
body_3.EnableCollision(False)
body_3.SetMass(1.0)
body_3.SetInertiaXX(chrono.ChVector3d(1, 1, 1))


box_3_shape = chrono.ChVisualShapeBox(1, 1, 1)
box_3_shape.SetColor(chrono.ChColor(0, 0, 0.6)) 
body_3.AddVisualShape(box_3_shape)


spring_2 = chrono.ChLinkTSDA()

spring_2.Initialize(body_1, body_2, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
spring_2.SetRestLength(rest_length)
spring_2.SetSpringCoefficient(spring_coef)
spring_2.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_2)


spring_2_shape = chrono.ChVisualShapeSpring(0.05, 80, 15)
spring_2.AddVisualShape(spring_2_shape)


spring_3 = chrono.ChLinkTSDA()

spring_3.Initialize(body_2, body_3, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
spring_3.SetRestLength(rest_length)
spring_3.SetSpringCoefficient(spring_coef)
spring_3.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_3)


spring_3_shape = chrono.ChVisualShapeSpring(0.05, 80, 15)
spring_3.AddVisualShape(spring_3_shape)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ChLinkTSDA 3-Body Chain Demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()

vis.AddCamera(chrono.ChVector3d(-1, -4.5, 10), chrono.ChVector3d(-1, -4.5, 0))
vis.AddTypicalLights()


time_step = 1e-3
while vis.Run():
    vis.BeginScene()
    vis.Render()
    
    
    vis.EndScene()
    sys.DoStepDynamics(time_step)