import pychrono as chrono
import pychrono.irrlicht as chronoirr
from pychrono import ChLinkTSDA

# Define the spring parameters
rest_length = 1.5
spring_coef = 50
damping_coef = 1

# Create the Chrono::Engine system
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

# Create and configure the ground body
ground = chrono.ChBody()
sys.AddBody(ground)
ground.SetFixed(True)
ground.EnableCollision(False)

# Add visual shapes to the ground
sph_1 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_1, chrono.ChFramed(chrono.ChVector3d(-1, 0, 0)))

sph_2 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_2, chrono.ChFramed(chrono.ChVector3d(1, 0, 0)))

# Create and configure the first body
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

# Create the first spring between body_1 and ground
spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(-1, 0, 0))
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_1)

# Attach a visualization asset to the spring
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

# Create and configure the second body
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

# Define a custom force functor for the second spring
class MySpringForce(chrono.ForceFunctorBase):
    def __init__(self, spring_coef, damping_coef, rest_length):
        super(MySpringForce, self).__init__()
        self.spring_coef = spring_coef
        self.damping_coef = damping_coef
        self.rest_length = rest_length

    def ComputeForce(self, time, dt, link, F):
        # Get the spring length
        spring_length = link.GetDistance()

        # Compute the spring force
        spring_force = self.spring_coef * (spring_length - self.rest_length)

        # Compute the damping force
        damping_force = self.damping_coef * link.GetVelocity()

        # Set the total force
        F = chrono.ChVector3d(spring_force + damping_force, 0, 0)

# Create the second spring between body_2 and ground using the custom force functor
spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_2, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(1, 0, 0))
spring_2.SetRestLength(rest_length)
spring_2.SetCustomForceFunctor(MySpringForce(spring_coef, damping_coef, rest_length))
sys.AddLink(spring_2)

# Attach a visualization asset to the second spring
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

# Create the visualization system
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