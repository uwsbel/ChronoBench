import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Custom force functor class
class MySpringForce(chrono.ForceFunctor):
    def __init__(self):
        super().__init__()
        self.spring_coef = 50
        self.damping_coef = 1

    def Evaluate(self, F, dt):
        # Get relative position between the two bodies
        delta = self.GetLink().GetRelativePosition()
        # Get relative velocity between the two bodies
        velocity = self.GetLink().GetRelativeVelocity()
        # Calculate spring force
        F spring_force = -self.spring_coef * delta - self.damping_coef * velocity
        F.force = spring_force

rest_length = 1.5
spring_coef = 50
damping_coef = 1

sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

ground = chrono.ChBody()
sys.AddBody(ground)
ground.SetFixed(True)
ground.EnableCollision(False)

# Add visual shapes to ground
sph_1 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_1, chrono.ChFramed(chrono.ChVector3d(-1, 0, 0)))

sph_2 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_2, chrono.ChFramed(chrono.ChVector3d(1, 0, 0)))

# Create body_1
body_1 = chrono.ChBody()
sys.AddBody(body_1)
body_1.SetPos(chrono.ChVector3d(-1, -3, 0))
body_1.SetFixed(False)
body_1.EnableCollision(False)
body_1.SetMass(1)
body_1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

# Attach a visualization asset to body_1
box_1 = chrono.ChVisualShapeBox(1, 1, 1)
box_1.SetColor(chrono.ChColor(0.6, 0, 0))
body_1.AddVisualShape(box_1)

# Create body_2
body_2 = chrono.ChBody()
sys.AddBody(body_2)
body_2.SetPos(chrono.ChVector3d(1, -3, 0))
body_2.SetFixed(False)
body_2.EnableCollision(False)
body_2.SetMass(1)
body_2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

# Attach a visualization asset to body_2
box_2 = chrono.ChVisualShapeBox(1, 1, 1)
box_2.SetColor(chrono.ChColor(0, 0, 0.6))
body_2.AddVisualShape(box_2)

# Create spring_1 between body_1 and ground
spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(-1, 0, 0))
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_1)

# Attach a visualization asset to spring_1
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

# Create spring_2 between body_2 and ground using custom force functor
spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_2, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(1, 0, 0))
spring_2.SetRestLength(rest_length)
sys.AddLink(spring_2)

# Create and attach custom force functor to spring_2
my_force = MySpringForce()
my_force.spring_coef = spring_coef
my_force.damping_coef = damping_coef
spring_2.SetForceFunctor(my_force)

# Attach a visualization asset to spring_2
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

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