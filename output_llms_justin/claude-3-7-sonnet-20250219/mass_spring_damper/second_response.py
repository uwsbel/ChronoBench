import pychrono as chrono
import pychrono.irrlicht as chronoirr


# Define a custom force functor for spring forces
class MySpringForce(chrono.ForceFunctor):
    def __init__(self, spring_k, damping_c, rest_l):
        super(MySpringForce, self).__init__()
        self.spring_k = spring_k
        self.damping_c = damping_c
        self.rest_l = rest_l
        
    def evaluate(self, distance, velocity, t):
        # Calculate spring force: F = -k*(x-L) - c*v
        force = -self.spring_k * (distance - self.rest_l) - self.damping_c * velocity
        return force


# Spring parameters
rest_length = 1.5
spring_coef = 50
damping_coef = 1


# Create the mechanical system
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector(0, -9.81, 0))  # Set gravity

# Create the ground body
ground = chrono.ChBody()
sys.AddBody(ground)
ground.SetFixed(True)
ground.SetBodyFixed(True)
ground.EnableCollision(False)

# Add visual elements to the ground
sph_1 = chrono.ChVisualShapeSphere(0.1)
sph_1.SetColor(chrono.ChColor(0, 0.6, 0))
ground.AddVisualShape(sph_1, chrono.ChFrame(chrono.ChVector(-1, 0, 0)))

# Add the second visual sphere to the ground at (1, 0, 0)
sph_2 = chrono.ChVisualShapeSphere(0.1)
sph_2.SetColor(chrono.ChColor(0, 0.6, 0))
ground.AddVisualShape(sph_2, chrono.ChFrame(chrono.ChVector(1, 0, 0)))

# Create the first body
body_1 = chrono.ChBody()
sys.AddBody(body_1)
body_1.SetPos(chrono.ChVector(-1, -3, 0))
body_1.SetFixed(False)
body_1.EnableCollision(False)
body_1.SetMass(1)
body_1.SetInertiaXX(chrono.ChVector(1, 1, 1))

# Attach a visualization asset to body_1
box_1 = chrono.ChVisualShapeBox(1, 1, 1)
box_1.SetColor(chrono.ChColor(0.6, 0, 0))
body_1.AddVisualShape(box_1)

# Create the second body (mirroring body_1)
body_2 = chrono.ChBody()
sys.AddBody(body_2)
body_2.SetPos(chrono.ChVector(1, -3, 0))
body_2.SetFixed(False)
body_2.EnableCollision(False)
body_2.SetMass(1)
body_2.SetInertiaXX(chrono.ChVector(1, 1, 1))

# Attach a visualization asset to body_2
box_2 = chrono.ChVisualShapeBox(1, 1, 1)
box_2.SetColor(chrono.ChColor(0, 0, 0.6))
body_2.AddVisualShape(box_2)

# Create the first spring between body_1 and ground using direct coefficients
spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground, 
                   True, chrono.ChVector(0, 0.5, 0), chrono.ChVector(-1, 0, 0),
                   False, rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_1)

# Attach a visualization asset to the spring
spring_vis_1 = chrono.ChVisualShapeSpring(0.05, 80, 15)
spring_vis_1.SetColor(chrono.ChColor(0.6, 0.2, 0.2))
spring_1.AddVisualShape(spring_vis_1)

# Create the second spring between body_2 and ground using the custom force functor
spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_2, ground, 
                   True, chrono.ChVector(0, 0.5, 0), chrono.ChVector(1, 0, 0),
                   False, rest_length)

# Create and set the custom force functor
my_force_functor = MySpringForce(spring_coef, damping_coef, rest_length)
spring_2.RegisterForceFunctor(my_force_functor)
sys.AddLink(spring_2)

# Attach a visualization asset to the second spring
spring_vis_2 = chrono.ChVisualShapeSpring(0.05, 80, 15)
spring_vis_2.SetColor(chrono.ChColor(0.2, 0.2, 0.6))
spring_2.AddVisualShape(spring_vis_2)

# Set up the visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ChLinkTSDA demo with Custom Force Functor')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector(0, -2, 6))
vis.AddTypicalLights()

# Run the simulation
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)