import pychrono as chrono
import pychrono.irrlicht as chronoirr


# Constants
rest_length = 1.5
spring_coef = 50
damping_coef = 1


# Create the simulation system
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, 0))  # Corrected to ChVectorD


# Create the ground body
ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000)  # Modified to ChBodyEasyBox for a box shape
sys.AddBody(ground)
ground.SetFixed(True)
ground.EnableCollision(False)

# Add visual shapes to the ground
sph_1 = chrono.ChVisualShapeSphere(ground, 0.1)
sph_1.SetColor(chrono.ChColor(0.6, 0.6, 0.6))
sph_1.SetPosition(chrono.ChVectorD(-1, 0, 0))
ground.AddVisualShape(sph_1)

sph_2 = chrono.ChVisualShapeSphere(ground, 0.1)
sph_2.SetColor(chrono.ChColor(0.6, 0.6, 0.6))
sph_2.SetPosition(chrono.ChVectorD(1, 0, 0))
ground.AddVisualShape(sph_2)


# Create body_1
body_1 = chrono.ChBodyEasyBox(sys, 1, 1, 1, 1)  # Modified to ChBodyEasyBox for a box shape
sys.AddBody(body_1)
body_1.SetPos(chrono.ChVectorD(-1, -3, 0))
body_1.SetFixed(False)
body_1.EnableCollision(False)

# Add visual shape to body_1
box_1 = chrono.ChVisualShapeBox(body_1, 1, 1, 1)
box_1.SetColor(chrono.ChColor(0.6, 0, 0))
body_1.AddVisualShape(box_1)


# Create body_2
body_2 = chrono.ChBodyEasyBox(sys, 1, 1, 1, 1)  # Modified to ChBodyEasyBox for a box shape
sys.AddBody(body_2)
body_2.SetPos(chrono.ChVectorD(1, -3, 0))
body_2.SetFixed(False)
body_2.EnableCollision(False)

# Add visual shape to body_2
box_2 = chrono.ChVisualShapeBox(body_2, 1, 1, 1)
box_2.SetColor(chrono.ChColor(0, 0.6, 0))
body_2.AddVisualShape(box_2)


# Custom force functor for spring force
class MySpringForce(chrono.ForceFunctor):
    def __init__(self, spring_coef, damping_coef):
        super().__init__()
        self.spring_coef = spring_coef
        self.damping_coef = damping_coef

    def Evaluate(self, time, object1, object2, link_info):
        # Calculate the relative position and velocity
        rel_pos = object2.GetPos() - object1.GetPos()
        rel_vel = object2.GetPos_dt() - object1.GetPos_dt()

        # Calculate the spring force
        force = -self.spring_coef * rel_pos - self.damping_coef * rel_vel

        return force


# Create spring_1 using direct spring and damping coefficients
spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground, True, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(-1, 0, 0))
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_1)

# Add visual shape to spring_1
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))


# Create spring_2 using the custom force functor
spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_2, ground, True, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(1, 0, 0))
spring_2.SetRestLength(rest_length)
spring_2.SetForceFunctor(MySpringForce(spring_coef, damping_coef))
sys.AddLink(spring_2)

# Add visual shape to spring_2
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))


# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ChLinkTSDA demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, 6))
vis.AddTypicalLights()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)