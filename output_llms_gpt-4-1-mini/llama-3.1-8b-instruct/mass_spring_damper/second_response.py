import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Define custom force functor
class MySpringForce(chrono.ForceFunctor):
    def __init__(self, spring_coef, damping_coef):
        self.spring_coef = spring_coef
        self.damping_coef = damping_coef

    def GetForce(self, body, point_on_body, rel_pos, rel_v):
        # Calculate spring force
        spring_force = self.spring_coef * rel_pos
        # Calculate damping force
        damping_force = -self.damping_coef * rel_v
        return spring_force + damping_force

# Define parameters
rest_length = 1.5
spring_coef = 50
damping_coef = 1

# Create the simulation system
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

# Create the ground
ground = chrono.ChBody()
sys.AddBody(ground)
ground.SetFixed(True)
ground.EnableCollision(False)

# Create visual sphere shape for ground
sph_1 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_1, chrono.ChFramed(chrono.ChVector3d(-1, 0, 0)))

# Create visual sphere shape for ground (mirrored)
sph_2 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_2, chrono.ChFramed(chrono.ChVector3d(1, 0, 0)))

# Create body 1
body_1 = chrono.ChBody()
sys.AddBody(body_1)
body_1.SetPos(chrono.ChVector3d(-1, -3, 0))
body_1.SetFixed(False)
body_1.EnableCollision(False)
body_1.SetMass(1)
body_1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

# Attach visualization asset to body 1
box_1 = chrono.ChVisualShapeBox(1, 1, 1)
box_1.SetColor(chrono.ChColor(0.6, 0, 0))
body_1.AddVisualShape(box_1)

# Create body 2 (mirrored)
body_2 = chrono.ChBody()
sys.AddBody(body_2)
body_2.SetPos(chrono.ChVector3d(1, -3, 0))
body_2.SetFixed(False)
body_2.EnableCollision(False)
body_2.SetMass(1)
body_2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

# Attach visualization asset to body 2
box_2 = chrono.ChVisualShapeBox(1, 1, 1)
box_2.SetColor(chrono.ChColor(0.6, 0, 0))
body_2.AddVisualShape(box_2)

# Create spring 1 between body 1 and ground
spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(-1, 0, 0))
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_1)

# Attach visualization asset to spring 1
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

# Create spring 2 between body 2 and ground using custom force functor
spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_2, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(1, 0, 0))
spring_2.SetForceFunctor(MySpringForce(spring_coef, damping_coef))
sys.AddLink(spring_2)

# Attach visualization asset to spring 2
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