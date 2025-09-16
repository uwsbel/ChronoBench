import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Constants
rest_length = 1.5
spring_coef = 50
damping_coef = 1

# Create the system
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Standard gravity

# Create the ground body
ground = chrono.ChBody()
sys.AddBody(ground)
ground.SetFixed(True)
ground.EnableCollision(False)

# Add visual sphere to ground at (-1, 0, 0)
sph_1 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_1, chrono.ChFrame(chrono.ChVector3d(-1, 0, 0)))

# Add another visual sphere to ground at (1, 0, 0)
sph_2 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_2, chrono.ChFrame(chrono.ChVector3d(1, 0, 0)))

# Create body 1
body_1 = chrono.ChBody()
sys.AddBody(body_1)
body_1.SetPos(chrono.ChVector3d(-1, -3, 0))
body_1.SetFixed(False)
body_1.EnableCollision(False)
body_1.SetMass(1)
body_1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

# Attach a visualization asset to body 1
box_1 = chrono.ChVisualShapeBox(1, 1, 1)
box_1.SetColor(chrono.ChColor(0.6, 0, 0))
body_1.AddVisualShape(box_1)

# Create body 2, mirroring body 1
body_2 = chrono.ChBody()
sys.AddBody(body_2)
body_2.SetPos(chrono.ChVector3d(1, -3, 0))  # Mirrored position
body_2.SetFixed(False)
body_2.EnableCollision(False)
body_2.SetMass(1)
body_2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

# Attach a visualization asset to body 2
box_2 = chrono.ChVisualShapeBox(1, 1, 1)
box_2.SetColor(chrono.ChColor(0, 0.6, 0))  # Different color for body 2
body_2.AddVisualShape(box_2)

# Custom force functor for spring
class MySpringForce(chrono.ForceFunctor):
    def __init__(self, spring_coef, damping_coef):
        self.spring_coef = spring_coef
        self.damping_coef = damping_coef

    def evaluate(self, force, torque, body1, body2, rel_pos, rel_vel, t):
        dist = rel_pos.Length()
        force = -self.spring_coef * (dist - rest_length) * rel_pos.Normalize() - self.damping_coef * rel_vel
        torque = chrono.ChVector3d(0, 0, 0)
        return

# Create spring 1 between body_1 and ground using direct coefficients
spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(-1, 0, 0))
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_1)

# Add visualization for spring 1
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

# Create spring 2 between body_2 and ground using custom force functor
spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_2, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(1, 0, 0))
custom_force = MySpringForce(spring_coef, damping_coef)
spring_2.SetForceFunctor(custom_force)
spring_2.SetRestLength(rest_length)
sys.AddLink(spring_2)

# Add visualization for spring 2
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

# Irrlicht application
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ChLinkTSDA demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 6))
vis.AddTypicalLights()

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)