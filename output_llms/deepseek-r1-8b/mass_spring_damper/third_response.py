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

# Create body_1
body_1 = chrono.ChBody()
sys.AddBody(body_1)
body_1.SetPos(chrono.ChVector3d(-1, -3, 0))
body_1.SetFixed(False)
body_1.EnableCollision(False)
body_1.SetMass(1)
body_1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

# Create body_2
body_2 = chrono.ChBody()
sys.AddBody(body_2)
body_2.SetPos(chrono.ChVector3d(-1, 1, 0))
body_2.SetMass(1)
body_2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

# Create body_3
body_3 = chrono.ChBody()
sys.AddBody(body_3)
body_3.SetPos(chrono.ChVector3d(-1, 2, 0))
body_3.SetMass(1)
body_3.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

# Create spring between body_1 and body_2
spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_1, body_2, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(-1, 0, 0))
spring_2.SetRestLength(rest_length)
spring_2.SetSpringCoefficient(spring_coef)
spring_2.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_2)

# Create spring between body_2 and body_3
spring_3 = chrono.ChLinkTSDA()
spring_3.Initialize(body_2, body_3, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(-1, 0, 0))
spring_3.SetRestLength(rest_length)
spring_3.SetSpringCoefficient(spring_coef)
spring_3.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_3)

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ChLinkTSDA demo with Multiple Springs')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 6))
vis.AddTypicalLights()

# Add visualization shapes
# Body 1 visualization
box_1 = chrono.ChVisualShapeBox(1, 1, 1)
box_1.SetColor(chrono.ChColor(0.6, 0, 0))
body_1.AddVisualShape(box_1)

# Body 2 visualization
box_2 = chrono.ChVisualShapeBox(1, 1, 1)
box_2.SetColor(chrono.ChColor(0, 0.6, 0))
body_2.AddVisualShape(box_2)

# Body 3 visualization
box_3 = chrono.ChVisualShapeBox(1, 1, 1)
box_3.SetColor(chrono.ChColor(0.6, 0, 0))
body_3.AddVisualShape(box_3)

# Spring visualization
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))
spring_3.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)