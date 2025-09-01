import pychrono as chrono
import pychrono.irrlicht as chronoirr


rest_length = 0.5  
spring_coef = 100
damping_coef = 2


sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

ground = chrono.ChBody()
sys.AddBody(ground)
ground.SetFixed(True)
ground.EnableCollision(False)


body_1 = chrono.ChBody()
sys.AddBody(body_1)
body_1.SetPos(chrono.ChVector3d(-1, -3, 0))
body_1.SetMass(1)
body_1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
body_1.EnableCollision(False)


body_2 = chrono.ChBody()
sys.AddBody(body_2)
body_2.SetPos(chrono.ChVector3d(0, -3, 0))
body_2.SetMass(1)
body_2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
body_2.EnableCollision(False)


body_3 = chrono.ChBody()
sys.AddBody(body_3)
body_3.SetPos(chrono.ChVector3d(1, -3, 0))
body_3.SetMass(1)
body_3.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
body_3.EnableCollision(False)


box_1 = chrono.ChVisualShapeBox(1, 1, 1)
box_1.SetColor(chrono.ChColor(0.6, 0, 0))
body_1.AddVisualShape(box_1)


spring_12 = chrono.ChLinkTSDA()
spring_12.Initialize(body_1, body_2, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(1, 0, 0))
spring_12.SetRestLength(rest_length)
spring_12.SetSpringCoefficient(spring_coef)
spring_12.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_12)


spring_vis_12 = chrono.ChVisualShapeSpring(0.05, 80, 15)
spring_12.AddVisualShape(spring_vis_12)


spring_23 = chrono.ChLinkTSDA()
spring_23.Initialize(body_2, body_3, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(1, 0, 0))
spring_23.SetRestLength(rest_length)
spring_23.SetSpringCoefficient(spring_coef)
spring_23.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_23)


spring_vis_23 = chrono.ChVisualShapeSpring(0.05, 80, 15)
spring_23.AddVisualShape(spring_vis_23)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ChLinkTSDA demo with Extended Bodies')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 6))
vis.AddTypicalLights()

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-4)