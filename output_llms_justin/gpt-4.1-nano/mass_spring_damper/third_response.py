import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Parameters
rest_length = 1.5
spring_coef = 50
damping_coef = 1

# Create system
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

# Create ground body
ground = chrono.ChBody()
sys.AddBody(ground)
ground.SetFixed(True)
ground.EnableCollision(False)

# Add visual shape to ground (optional)
sph_1 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_1, chrono.ChFramed(chrono.ChVector3d(-1, 0, 0)))

# Create body_1
body_1 = chrono.ChBody()
sys.AddBody(body_1)
body_1.SetPos(chrono.ChVector3d(-1, -3, 0))
body_1.SetFixed(False)
body_1.EnableCollision(False)
body_1.SetMass(1)
body_1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

# Attach a visual shape to body_1
box_1 = chrono.ChVisualShapeBox(1, 1, 1)
box_1.SetColor(chrono.ChColor(0.6, 0, 0))
body_1.AddVisualShape(box_1)

# Create new bodies: body_2 and body_3
# Body 2
body_2 = chrono.ChBody()
sys.AddBody(body_2)
body_2.SetPos(chrono.ChVector3d(0, -3, 0))
body_2.SetFixed(False)
body_2.EnableCollision(False)
body_2.SetMass(1)
body_2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
# Attach visual shape
box_2 = chrono.ChVisualShapeBox(1, 1, 1)
box_2.SetColor(chrono.ChColor(0, 0.6, 0))
body_2.AddVisualShape(box_2)

# Body 3
body_3 = chrono.ChBody()
sys.AddBody(body_3)
body_3.SetPos(chrono.ChVector3d(1, -3, 0))
body_3.SetFixed(False)
body_3.EnableCollision(False)
body_3.SetMass(1)
body_3.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
# Attach visual shape
box_3 = chrono.ChVisualShapeBox(1, 1, 1)
box_3.SetColor(chrono.ChColor(0, 0, 0.6))
body_3.AddVisualShape(box_3)

# Springs between bodies
# Spring between body_1 and ground (already present), but to add between body_1 and body_2
spring_1_2 = chrono.ChLinkTSDA()
spring_1_2.Initialize(body_1, body_2, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
spring_1_2.SetRestLength(rest_length)
spring_1_2.SetSpringCoefficient(spring_coef)
spring_1_2.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_1_2)
spring_1_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

# Spring between body_2 and body_3
spring_2_3 = chrono.ChLinkTSDA()
spring_2_3.Initialize(body_2, body_3, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
spring_2_3.SetRestLength(rest_length)
spring_2_3.SetSpringCoefficient(spring_coef)
spring_2_3.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_2_3)
spring_2_3.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

# Visualization using Irrlicht
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ChLinkTSDA demo with added bodies and springs')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 6))
vis.AddTypicalLights()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)