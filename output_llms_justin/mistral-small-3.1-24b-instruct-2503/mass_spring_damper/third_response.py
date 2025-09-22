import pychrono as chrono
import pychrono.irrlicht as chronoirr

rest_length = 1.5
spring_coef = 50
damping_coef = 1

# Create the Chrono::Engine physical system
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

# Create the ground body
ground = chrono.ChBodyEasyBox(10, 10, 0.5, 1000)
ground.SetPos(chrono.ChVector3d(0, 0, -0.25))
ground.SetFixed(True)
ground.EnableCollision(False)
sys.AddBody(ground)

# Create body_1
body_1 = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True)
body_1.SetPos(chrono.ChVector3d(-1, -3, 0))
body_1.SetFixed(False)
body_1.EnableCollision(False)
sys.AddBody(body_1)

# Attach a visualization asset to body_1
box_1 = chrono.ChVisualShapeBox(1, 1, 1)
box_1.SetColor(chrono.ChColor(0.6, 0, 0))
body_1.AddVisualShape(box_1)

# Create the spring between body_1 and ground
spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_1)

# Attach a visualization asset to the spring
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

# Create body_2
body_2 = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True)
body_2.SetPos(chrono.ChVector3d(0, -3, 0))
body_2.SetFixed(False)
body_2.EnableCollision(False)
sys.AddBody(body_2)

# Attach a visualization asset to body_2
box_2 = chrono.ChVisualShapeBox(1, 1, 1)
box_2.SetColor(chrono.ChColor(0, 0.6, 0))
body_2.AddVisualShape(box_2)

# Create the spring between body_1 and body_2
spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_1, body_2, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
spring_2.SetRestLength(rest_length)
spring_2.SetSpringCoefficient(spring_coef)
spring_2.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_2)

# Attach a visualization asset to the spring
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

# Create body_3
body_3 = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True)
body_3.SetPos(chrono.ChVector3d(1, -3, 0))
body_3.SetFixed(False)
body_3.EnableCollision(False)
sys.AddBody(body_3)

# Attach a visualization asset to body_3
box_3 = chrono.ChVisualShapeBox(1, 1, 1)
box_3.SetColor(chrono.ChColor(0, 0, 0.6))
body_3.AddVisualShape(box_3)

# Create the spring between body_2 and body_3
spring_3 = chrono.ChLinkTSDA()
spring_3.Initialize(body_2, body_3, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
spring_3.SetRestLength(rest_length)
spring_3.SetSpringCoefficient(spring_coef)
spring_3.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_3)

# Attach a visualization asset to the spring
spring_3.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ChLinkTSDA demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 6))
vis.AddTypicalLights()

# Run the simulation
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)