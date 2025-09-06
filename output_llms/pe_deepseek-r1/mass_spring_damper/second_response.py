import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Custom force functor implementation
class MySpringForce(chrono.ForceFunctor):
    def __init__(self, spring_coef, damping_coef):
        super().__init__()
        self.spring_coef = spring_coef
        self.damping_coef = damping_coef
        
    def Evaluate(self, time, rest_length, length, vel, link):
        return -self.spring_coef * (length - rest_length) - self.damping_coef * vel

# Simulation parameters
rest_length = 1.5
spring_coef = 50
damping_coef = 1

# System setup
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

# Ground body
ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)
ground.EnableCollision(False)

# Ground visual elements
sph_1 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_1, chrono.ChFrameD(chrono.ChVector3d(-1, 0, 0)))

sph_2 = chrono.ChVisualShapeSphere(0.1)  # Added mirror sphere
ground.AddVisualShape(sph_2, chrono.ChFrameD(chrono.ChVector3d(1, 0, 0)))  # Position (1,0,0)

# Body 1
body_1 = chrono.ChBody()
sys.Add(body_1)
body_1.SetPos(chrono.ChVector3d(-1, -3, 0))
body_1.SetFixed(False)
body_1.EnableCollision(False)
body_1.SetMass(1)
body_1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

# Body 1 visual
box_1 = chrono.ChVisualShapeBox(1, 1, 1)
box_1.SetColor(chrono.ChColor(0.6, 0, 0))
body_1.AddVisualShape(box_1)

# Body 2 (mirrored configuration)
body_2 = chrono.ChBody()
sys.Add(body_2)
body_2.SetPos(chrono.ChVector3d(1, -3, 0))  # Mirror position
body_2.SetFixed(False)
body_2.EnableCollision(False)
body_2.SetMass(1)
body_2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

# Body 2 visual
box_2 = chrono.ChVisualShapeBox(1, 1, 1)
box_2.SetColor(chrono.ChColor(0, 0.6, 0))  # Different color for distinction
body_2.AddVisualShape(box_2)

# Spring 1 (direct coefficients)
spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground, False, 
                   chrono.ChVector3d(0, 0, 0),  # Body 1 attachment
                   chrono.ChVector3d(-1, 0, 0))  # Ground attachment
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_1)

# Spring 1 visualization
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

# Spring 2 (custom force functor)
spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_2, ground, False, 
                   chrono.ChVector3d(0, 0, 0),  # Body 2 attachment
                   chrono.ChVector3d(1, 0, 0))   # Ground attachment
spring_2.SetRestLength(rest_length)
spring_2.RegisterForceFunctor(MySpringForce(spring_coef, damping_coef))  # Custom force
sys.AddLink(spring_2)

# Spring 2 visualization
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

# Visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Enhanced ChLinkTSDA Demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -6, 8))  # Adjusted for better view
vis.AddTypicalLights()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)