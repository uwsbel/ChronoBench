import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Define constants
rest_length = 1.5
spring_coef = 50
damping_coef = 1

# Custom force functor class for spring force
class MySpringForce(chrono.ForceFunctor):
    def __init__(self, spring_coefficient, damping_coefficient):
        super().__init__()
        self.spring_coefficient = spring_coefficient
        self.damping_coefficient = damping_coefficient
        self.spring_force = 0.0

    def Evaluate(self, time, bodyA, bodyB, rel_pos, rel_vel):
        # Calculate spring force based on displacement
        displacement = rel_pos.Length() - rest_length
        # Calculate relative velocity along the spring axis
        rel_vel_along_axis = rel_vel.Dot(rel_pos.GetNormalized())
        # Hooke's law with damping
        force_magnitude = -self.spring_coefficient * displacement - self.damping_coefficient * rel_vel_along_axis
        self.spring_force = force_magnitude
        return force_magnitude

# Initialize the system
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

# Create ground body
ground = chrono.ChBody()
sys.AddBody(ground)
ground.SetFixed(True)
ground.EnableCollision(False)

# Add visual sphere to ground at (-1, 0, 0)
sph_1 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_1, chrono.ChFramed(chrono.ChVector3d(-1, 0, 0)))

# Create first dynamic body (body_1)
body_1 = chrono.ChBody()
sys.AddBody(body_1)
body_1.SetPos(chrono.ChVector3d(-1, -3, 0))
body_1.SetFixed(False)
body_1.EnableCollision(False)
body_1.SetMass(1)
body_1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

# Attach visual shape to body_1
box_1 = chrono.ChVisualShapeBox()
box_1.GetBoxGeometry().Size = chrono.ChVector3d(0.5, 0.5, 0.5)
box_1.SetColor(chrono.ChColor(0.6, 0, 0))
body_1.AddVisualShape(box_1)

# Create second dynamic body (body_2), similar to body_1
body_2 = chrono.ChBody()
sys.AddBody(body_2)
body_2.SetPos(chrono.ChVector3d(1, -3, 0))
body_2.SetFixed(False)
body_2.EnableCollision(False)
body_2.SetMass(1)
body_2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

# Attach visual shape to body_2
box_2 = chrono.ChVisualShapeBox()
box_2.GetBoxGeometry().Size = chrono.ChVector3d(0.5, 0.5, 0.5)
box_2.SetColor(chrono.ChColor(0, 0.6, 0))
body_2.AddVisualShape(box_2)

# Add visual sphere to ground at (1, 0, 0)
sph_2 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_2, chrono.ChFramed(chrono.ChVector3d(1, 0, 0)))

# Create spring between body_1 and ground
spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(-1, 0, 0))
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_1)

# Attach visual shape for spring_1
spring_visual_1 = chrono.ChVisualShapeSpring(0.05, 80, 15)
spring_1.AddVisualShape(spring_visual_1)

# Create a custom force functor instance for the second spring
my_spring_force = MySpringForce(spring_coef, damping_coef)

# Create spring between body_2 and ground using the custom force functor
spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_2, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(1, 0, 0))
spring_2.SetRestLength(rest_length)

# Attach the custom force functor to spring_2
spring_2.SetForceFunctor(my_spring_force)
sys.AddLink(spring_2)

# Attach visual shape for spring_2
spring_visual_2 = chrono.ChVisualShapeSpring(0.05, 80, 15)
spring_2.AddVisualShape(spring_visual_2)

# Set up Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ChLinkTSDA demo with Custom Force Functor')
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