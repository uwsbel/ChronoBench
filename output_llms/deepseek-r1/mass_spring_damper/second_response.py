import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Parameters
rest_length = 1.5
spring_coef = 50
damping_coef = 1

# Custom force functor implementation
class MySpringForce(chrono.ChLinkTSDA.ForceFunctor):
    def __init__(self, spring_coef, damping_coef):
        super().__init__()
        self.spring_coef = spring_coef
        self.damping_coef = damping_coef
        
    def evaluate(self, time, rest_length, length, vel, link):
        force = self.spring_coef * (rest_length - length) - self.damping_coef * vel
        return force

# System setup
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

# Ground body setup
ground = chrono.ChBody()
sys.AddBody(ground)
ground.SetFixed(True)
ground.EnableCollision(False)

# Add visual spheres to ground
sph1 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph1, chrono.ChFrameD(chrono.ChVector3d(-1, 0, 0)))
sph2 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph2, chrono.ChFrameD(chrono.ChVector3d(1, 0, 0)))

# Create first body (left)
body1 = chrono.ChBody()
sys.AddBody(body1)
body1.SetPos(chrono.ChVector3d(-1, -3, 0))
body1.SetFixed(False)
body1.EnableCollision(False)
body1.SetMass(1)
body1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

# First body visualization
box1 = chrono.ChVisualShapeBox(1, 1, 1)
box1.SetColor(chrono.ChColor(0.6, 0, 0))
body1.AddVisualShape(box1)

# Create second body (right)
body2 = chrono.ChBody()
sys.AddBody(body2)
body2.SetPos(chrono.ChVector3d(1, -3, 0))
body2.SetFixed(False)
body2.EnableCollision(False)
body2.SetMass(1)
body2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

# Second body visualization
box2 = chrono.ChVisualShapeBox(1, 1, 1)
box2.SetColor(chrono.ChColor(0, 0.6, 0))
body2.AddVisualShape(box2)

# Create spring between body1 and ground (direct coefficients)
spring1 = chrono.ChLinkTSDA()
spring1.Initialize(body1, ground, False, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(-1, 0, 0))
spring1.SetRestLength(rest_length)
spring1.SetSpringCoefficient(spring_coef)
spring1.SetDampingCoefficient(damping_coef)
sys.AddLink(spring1)
spring1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

# Create spring between body2 and ground (custom force functor)
spring2 = chrono.ChLinkTSDA()
spring2.Initialize(body2, ground, False, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(1, 0, 0))
spring2.SetRestLength(rest_length)
spring2.RegisterForceFunctor(MySpringForce(spring_coef, damping_coef))
sys.AddLink(spring2)
spring2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Modified ChLinkTSDA demo')
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