import pychrono as chrono
import pychrono.irrlicht as chronoirr


rest_length    = 1.5
spring_coef    = 50.0
damping_coef   = 1.0


sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVector3d(0, 0, 0))


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.EnableCollision(False)
sys.Add(ground)


marker = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(marker, chrono.ChFrameD(chrono.ChVector3d(-1, 0, 0)))


body_1 = chrono.ChBody()
body_1.SetPos(chrono.ChVector3d(-1, -3, 0))
body_1.SetMass(1.0)
body_1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
body_1.EnableCollision(False)
sys.Add(body_1)


box1 = chrono.ChVisualShapeBox(1, 1, 1)
box1.SetColor(chrono.ChColor(0.6, 0.0, 0.0))
body_1.AddVisualShape(box1)


body_2 = chrono.ChBody()
body_2.SetPos(chrono.ChVector3d( 0, -3, 0))
body_2.SetMass(1.0)
body_2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
body_2.EnableCollision(False)
sys.Add(body_2)

box2 = chrono.ChVisualShapeBox(1, 1, 1)
box2.SetColor(chrono.ChColor(0.0, 0.6, 0.0))
body_2.AddVisualShape(box2)


body_3 = chrono.ChBody()
body_3.SetPos(chrono.ChVector3d( 1, -3, 0))
body_3.SetMass(1.0)
body_3.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
body_3.EnableCollision(False)
sys.Add(body_3)

box3 = chrono.ChVisualShapeBox(1, 1, 1)
box3.SetColor(chrono.ChColor(0.0, 0.0, 0.6))
body_3.AddVisualShape(box3)


spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground,
                    True,
                    chrono.ChVector3d(0, 0, 0),     
                    chrono.ChVector3d(-1, 0, 0))    
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_1)

vis_spr1 = chrono.ChVisualShapeSpring(0.05, 80, 15)
spring_1.AddVisualShape(vis_spr1)


spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_1, body_2,
                    True,
                    chrono.ChVector3d(0, 0, 0),
                    chrono.ChVector3d(0, 0, 0))
spring_2.SetRestLength(rest_length)
spring_2.SetSpringCoefficient(spring_coef)
spring_2.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_2)

vis_spr2 = chrono.ChVisualShapeSpring(0.05, 80, 15)
spring_2.AddVisualShape(vis_spr2)


spring_3 = chrono.ChLinkTSDA()
spring_3.Initialize(body_2, body_3,
                    True,
                    chrono.ChVector3d(0, 0, 0),
                    chrono.ChVector3d(0, 0, 0))
spring_3.SetRestLength(rest_length)
spring_3.SetSpringCoefficient(spring_coef)
spring_3.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_3)

vis_spr3 = chrono.ChVisualShapeSpring(0.05, 80, 15)
spring_3.AddVisualShape(vis_spr3)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ChLinkTSDA Chain Demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 6))
vis.AddTypicalLights()


timestep = 1e-3
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(timestep)