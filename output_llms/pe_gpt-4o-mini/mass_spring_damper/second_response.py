import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Define custom force functor for spring force
class MySpringForce(chrono.ChForceFunctor):
    def __init__(self, spring_coef, damping_coef):
        super().__init__()
        self.spring_coef = spring_coef
        self.damping_coef = damping_coef

    def Evaluate(self, body, point, vel):
        # Implementing Hooke's law for spring force
        displacement = point - body.GetPos()
        spring_force = -self.spring_coef * displacement
        damping_force = -self.damping_coef * vel
        return spring_force + damping_force

# Spring parameters
rest_length = 1.5
spring_coef = 50
damping_coef = 1

# Create the simulation system
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

# Create ground body
ground = chrono.ChBody()
ground.SetFixed(True)
ground.EnableCollision(False)
sys.AddBody(ground)

# Add visual sphere to ground
sph_1 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_1, chrono.ChFramed(chrono.ChVector3d(-1, 0, 0)))

# Create first body
body_1 = chrono.ChBody()
body_1.SetPos(chrono.ChVector3d(-1, -3, 0))
body_1.SetFixed(False)
body_1.EnableCollision(False)
body_1.SetMass(1)
body_1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
sys.AddBody(body_1)

# Attach a visualization asset to body_1
box_1 = chrono.ChVisualShapeBox(1, 1, 1)
box_1.SetColor(chrono.ChColor(0.6, 0, 0))
body_1.AddVisualShape(box_1)

# Create the spring between body_1 and ground
spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(-1, 0, 0))
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_1)

# Attach a visualization asset for spring_1
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

# Create second body (body_2) mirroring body_1
body_2 = chrono.ChBody()
body_2.SetPos(chrono.ChVector3d(1, -3, 0))  # Position it at (1, -3, 0)
body_2.SetFixed(False)
body_2.EnableCollision(False)
body_2.SetMass(1)
body_2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
sys.AddBody(body_2)

# Attach a visualization asset to body_2
box_2 = chrono.ChVisualShapeBox(1, 1, 1)
box_2.SetColor(chrono.ChColor(0, 0.6, 0))  # Different color for distinction
body_2.AddVisualShape(box_2)

# Create and configure spring_2 using MySpringForce
spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_2, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(1, 0, 0))
spring_2.SetRestLength(rest_length)

# Create and set the custom force functor for spring_2
spring_force_functor = MySpringForce(spring_coef, damping_coef)
spring_2.SetForceFunctor(spring_force_functor)

sys.AddLink(spring_2)

# Add second visual sphere shape to ground
sph_2 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_2, chrono.ChFramed(chrono.ChVector3d(1, 0, 0)))

# Initialize visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ChLinkTSDA demo')
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