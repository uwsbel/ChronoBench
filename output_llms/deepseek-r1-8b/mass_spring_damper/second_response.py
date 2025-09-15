import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Custom Force Functor
class MySpringForce(chrono.ForceFunctor):
    def __init__(self, spring_coef, damping_coef):
        super().__init__()
        self.spring_coef = spring_coef
        self.damping_coef = damping_coef

    def ComputeForce(self, body, other, frame):
        # Calculate displacement relative to the other body
        pos = body.GetPos(frame) - other.GetPos(frame)
        # Compute spring force
        spring_force = self.spring_coef * (0.1 - pos.GetZ())
        # Damping force
        damping_force = -self.damping_coef * pos.GetVelocity(frame).GetZ()
        # Total force
        return chrono.ChVector3d(0, 0, spring_force + damping_force)

# Rest length, spring coefficient, damping coefficient
rest_length = 0.1
spring_coef = 50
damping_coef = 1

# Create system
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

# Ground setup
ground = chrono.ChBody()
sys.AddBody(ground)
ground.SetFixed(True)
ground.EnableCollision(False)

# Visual elements for ground
sph_1 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_1, chrono.ChFramed(chrono.ChVector3d(-1, 0, 0)))
sph_2 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_2, chrono.ChFramed(chrono.ChVector3d(1, 0, 0)))

# Body 1 setup
body_1 = chrono.ChBody()
sys.AddBody(body_1)
body_1.SetPos(chrono.ChVector3d(-1, -3, 0))
body_1.SetFixed(False)
body_1.EnableCollision(False)
body_1.SetMass(1)
body_1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

# Visual elements for body 1
box_1 = chrono.ChVisualShapeBox(1, 1, 1)
box_1.SetColor(chrono.ChColor(0.6, 0, 0))
body_1.AddVisualShape(box_1)

# Spring 1 setup (using default spring)
spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(-1, 0, 0))
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_1)

# Visual elements for spring 1
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

# Body 2 setup (mirror of body_1)
body_2 = chrono.ChBody()
sys.AddBody(body_2)
body_2.SetPos(chrono.ChVector3d(1, -3, 0))
body_2.SetFixed(False)
body_2.EnableCollision(False)
body_2.SetMass(1)
body_2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

# Visual elements for body 2
box_2 = chrono.ChVisualShapeBox(1, 1, 1)
box_2.SetColor(chrono.ChColor(0.6, 0, 0))
body_2.AddVisualShape(box_2)

# Spring 2 setup (using custom force functor)
class MySpringForce2(chrono.ForceFunctor):
    def __init__(self, spring_coef, damping_coef):
        super().__init__()
        self.spring_coef = spring_coef
        self.damping_coef = damping_coef

    def ComputeForce(self, body, other, frame):
        pos = body.GetPos(frame) - other.GetPos(frame)
        spring_force = self.spring_coef * (0.1 - pos.GetZ())
        damping_force = -self.damping_coef * pos.GetVelocity(frame).GetZ()
        return chrono.ChVector3d(0, 0, spring_force + damping_force)

spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_2, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(-1, 0, 0))
spring_2.SetRestLength(rest_length)
spring_2.SetSpringCoefficient(spring_coef)
spring_2.SetDampingCoefficient(damping_coef)
# Attach custom force functor
spring_2.SetForceFunctor(MySpringForce2(spring_coef, damping_coef))
sys.AddLink(spring_2)

# Visual elements for spring 2
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ChLinkTSDA demo with Custom Force')
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