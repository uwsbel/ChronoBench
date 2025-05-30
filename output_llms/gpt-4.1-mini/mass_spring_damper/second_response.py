import pychrono as chrono
import pychrono.irrlicht as chronoirr


rest_length = 1.5
spring_coef = 50
damping_coef = 1


# 1. Define the custom force functor for spring force/damping
class MySpringForce(chrono.ForceFunctor):
    def __init__(self, k, r, c, l0):
        super().__init__()
        self.k = k      # spring constant
        self.r = r      # not used here but kept for signature compatibility
        self.c = c      # damping coefficient
        self.l0 = l0    # rest length

    def __call__(self, time, rest_length, length, vel, functor_data=None):
        # time: current time, rest_length: the rest length of the spring,
        # length: current length, vel: velocity of length change
        # functor_data is an optional parameter
        # Hooke's law + damping: F = -k*(length - rest_length) - c*vel
        force = -self.k * (length - self.l0) - self.c * vel
        return force


sys = chrono.ChSystemNSC()
# Set gravity to zero vector as originally done
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

# Ground body setup
ground = chrono.ChBody()
sys.AddBody(ground)
ground.SetFixed(True)
ground.EnableCollision(False)

# Add first sphere visual shape to ground at (-1, 0, 0)
sph_1 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_1, chrono.ChFrameD(chrono.ChVectorD(-1, 0, 0)))  # corrected: ChFrameD (not ChFramed)

# 3. Add second sphere visual shape to ground at (1, 0, 0)
sph_2 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_2, chrono.ChFrameD(chrono.ChVectorD(1, 0, 0)))

# Create body_1
body_1 = chrono.ChBody()
sys.AddBody(body_1)
body_1.SetPos(chrono.ChVectorD(-1, -3, 0))
body_1.SetFixed(False)
body_1.EnableCollision(False)
body_1.SetMass(1)
body_1.SetInertiaXX(chrono.ChVectorD(1, 1, 1))

# Attach a box visual shape to body_1
box_1 = chrono.ChVisualShapeBox(1, 1, 1)
box_1.SetColor(chrono.ChColor(0.6, 0, 0))
body_1.AddVisualShape(box_1)

# 2. Create body_2 similar to body_1
body_2 = chrono.ChBody()
sys.AddBody(body_2)

# Place body_2 symmetrically at (1, -3, 0)
body_2.SetPos(chrono.ChVectorD(1, -3, 0))
body_2.SetFixed(False)
body_2.EnableCollision(False)
body_2.SetMass(1)
body_2.SetInertiaXX(chrono.ChVectorD(1, 1, 1))

# Attach a box visual shape to body_2 (color different to distinguish)
box_2 = chrono.ChVisualShapeBox(1, 1, 1)
box_2.SetColor(chrono.ChColor(0, 0, 0.6))  # blue color for distinction
body_2.AddVisualShape(box_2)

# 4. Create spring_1 between body_1 and ground with direct spring and damping coefficients
spring_1 = chrono.ChLinkTSDA()
# The two points are in local coordinates of body_1 and ground respectively
spring_1.Initialize(body_1, ground, True,
                    chrono.ChVectorD(0, 0, 0),    # attachment point on body_1 (local)
                    chrono.ChVectorD(-1, 0, 0))   # attachment point on ground (local)
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_1)

# Attach visualization asset to spring_1
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

# 4. Create spring_2 between body_2 and ground using the custom force functor MySpringForce
spring_2 = chrono.ChLinkTSDA()

# For spring_2, define attachment points:
# On body_2 local coords: (0,0,0)
# On ground local coords: (1,0,0)
spring_2.Initialize(body_2, ground, True,
                    chrono.ChVectorD(0, 0, 0),
                    chrono.ChVectorD(1, 0, 0))

spring_2.SetRestLength(rest_length)

# Create an instance of the custom force functor
my_spring_force = MySpringForce(k=spring_coef, r=rest_length, c=damping_coef, l0=rest_length)

spring_2.SetForceFunctor(my_spring_force)

sys.AddLink(spring_2)

# Attach visualization asset to spring_2 as well
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

# Visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ChLinkTSDA with Custom Force Functor Demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, 6))
vis.AddTypicalLights()

# Simulation loop
timestep = 1e-3
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(timestep)