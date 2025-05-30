import pychrono as chrono
import pychrono.irrlicht as chronoirr

# --- PARAMETERS ---
rest_length = 1.5
spring_coef = 50
damping_coef = 1

# --- SYSTEM SETUP ---
sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVector3d(0, 0, 0))  # Correct method name

# --- GROUND BODY ---
ground = chrono.ChBody()
ground.SetBodyFixed(True)  # Correct method name
ground.SetCollide(False)   # Correct method name
sys.AddBody(ground)

# --- GROUND VISUALS ---
sph_1 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_1, chrono.ChFrame(chrono.ChVector3d(-1, 0, 0)))  # Correct: ChFrame, not ChFramed

sph_2 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_2, chrono.ChFrame(chrono.ChVector3d(1, 0, 0)))   # New sphere at (1,0,0)

# --- BODY 1 ---
body_1 = chrono.ChBody()
body_1.SetPos(chrono.ChVector3d(-1, -3, 0))
body_1.SetBodyFixed(False)
body_1.SetCollide(False)
body_1.SetMass(1)
body_1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
sys.AddBody(body_1)

box_1 = chrono.ChVisualShapeBox(1, 1, 1)
box_1.SetColor(chrono.ChColor(0.6, 0, 0))
body_1.AddVisualShape(box_1)

# --- BODY 2 (mirrored) ---
body_2 = chrono.ChBody()
body_2.SetPos(chrono.ChVector3d(1, -3, 0))  # Mirrored position
body_2.SetBodyFixed(False)
body_2.SetCollide(False)
body_2.SetMass(1)
body_2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
sys.AddBody(body_2)

box_2 = chrono.ChVisualShapeBox(1, 1, 1)
box_2.SetColor(chrono.ChColor(0, 0, 0.6))
body_2.AddVisualShape(box_2)

# --- SPRING 1: Direct coefficients ---
spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(-1, 0, 0))
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))
sys.AddLink(spring_1)

# --- CUSTOM FORCE FUNCTOR ---
class MySpringForce(chrono.ForceFunctor):
    def __init__(self, k, c):
        chrono.ForceFunctor.__init__(self)
        self.k = k
        self.c = c

    # Override the __call__ method for Python
    def __call__(self, time, rest_length, length, vel, link):
        # F = -k*(x - x0) - c*v
        return -self.k * (length - rest_length) - self.c * vel

# --- SPRING 2: Custom force functor ---
spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_2, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(1, 0, 0))
spring_2.SetRestLength(rest_length)
my_force = MySpringForce(spring_coef, damping_coef)
spring_2.RegisterForceFunctor(my_force)
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))
sys.AddLink(spring_2)

# --- IRRLICHT VISUALIZATION ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ChLinkTSDA demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 8))
vis.AddTypicalLights()

# --- SIMULATION LOOP ---
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)