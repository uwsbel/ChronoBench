import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Parameters
rest_length = 1.5
spring_coef = 50
damping_coef = 1

# System setup
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

# Ground body
ground = chrono.ChBody()
sys.AddBody(ground)
ground.SetFixed(True)
ground.EnableCollision(False)

# Ground visualization
sph_1 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_1, chrono.ChFrameD(chrono.ChVector3d(-1, 0, 0)))  # Fixed frame initialization

# Body 1
body_1 = chrono.ChBody()
sys.AddBody(body_1)
body_1.SetPos(chrono.ChVector3d(-1, -3, 0))
body_1.SetFixed(False)
body_1.EnableCollision(False)
body_1.SetMass(1)
body_1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

# Body 1 visualization
box_1 = chrono.ChVisualShapeBox(1, 1, 1)
box_1.SetColor(chrono.ChColor(0.6, 0, 0))
body_1.AddVisualShape(box_1)

# Body 2
body_2 = chrono.ChBody()
sys.AddBody(body_2)
body_2.SetPos(chrono.ChVector3d(0, -3, 0))
body_2.SetFixed(False)
body_2.EnableCollision(False)
body_2.SetMass(1)
body_2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

# Body 2 visualization
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

# Body 3 visualization
box_3 = chrono.ChVisualShapeBox(1, 1, 1)
box_3.SetColor(chrono.ChColor(0, 0, 0.6))
body_3.AddVisualShape(box_3)

# Spring between body_1 and ground
spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground, False,  # Disable auto rest length
                    chrono.ChVector3d(0, 0, 0),  # Body 1 point
                    chrono.ChVector3d(-1, 0, 0))  # Ground point
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_1)
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

# Spring between body_1 and body_2
spring_12 = chrono.ChLinkTSDA()
spring_12.Initialize(body_1, body_2, False,
                     chrono.ChVector3d(0, 0, 0),  # Body 1 point
                     chrono.ChVector3d(0, 0, 0))  # Body 2 point
spring_12.SetRestLength(1.0)  # Distance between bodies
spring_12.SetSpringCoefficient(spring_coef)
spring_12.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_12)
spring_12.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

# Spring between body_2 and body_3
spring_23 = chrono.ChLinkTSDA()
spring_23.Initialize(body_2, body_3, False,
                     chrono.ChVector3d(0, 0, 0),  # Body 2 point
                     chrono.ChVector3d(0, 0, 0))  # Body 3 point
spring_23.SetRestLength(1.0)  # Distance between bodies
spring_23.SetSpringCoefficient(spring_coef)
spring_23.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_23)
spring_23.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

# Visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Multi-body Spring System')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -1, 8))  # Adjusted camera position
vis.AddTypicalLights()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)