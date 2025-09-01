import pychrono as chrono
import pychrono.irrlicht as chronoirr

# ------------------------------------------------------------------
# 1) Problem constants
# ------------------------------------------------------------------
rest_length   = 1.5
spring_coef   = 50.0
damping_coef  = 1.0

# ------------------------------------------------------------------
# 2) Custom Force Functor
#    This must subclass chrono.ForceFunctor and override Evaluate(...)
# ------------------------------------------------------------------
class MySpringForce(chrono.ForceFunctor):
    def __init__(self, k, c):
        super(MySpringForce, self).__init__()  
        self.k = k
        self.c = c

    # Signature matches the C++ ForceFunctor::Evaluate for ChLinkTSDA:
    # time, rest_length, cur_length, rel_vel, body_a, body_b, pt_a, pt_b
    def Evaluate(self, time, rest_length, length, vel, bodyA, bodyB, abs_pointA, abs_pointB):
        # Simple linear spring-damper law
        return -self.k * (length - rest_length) - self.c * vel

# ------------------------------------------------------------------
# 3) System + Ground body
# ------------------------------------------------------------------
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.EnableCollision(False)
sys.AddBody(ground)

# original sphere visual at (-1,0,0)
sph_1 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_1, chrono.ChFrame(chrono.ChVector3d(-1, 0, 0)))

# new sphere visual at (1,0,0)
sph_2 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_2, chrono.ChFrame(chrono.ChVector3d(1, 0, 0)))

# ------------------------------------------------------------------
# 4) First Mass (body_1) – exactly as before
# ------------------------------------------------------------------
body_1 = chrono.ChBody()
body_1.SetPos(chrono.ChVector3d(-1, -3, 0))
body_1.SetMass(1.0)
body_1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
body_1.EnableCollision(False)
sys.AddBody(body_1)

box_1 = chrono.ChVisualShapeBox(1, 1, 1)
box_1.SetColor(chrono.ChColor(0.6, 0.0, 0.0))
body_1.AddVisualShape(box_1)

# ------------------------------------------------------------------
# 5) Second Mass (body_2) – mirror of body_1 at +1 in x
# ------------------------------------------------------------------
body_2 = chrono.ChBody()
body_2.SetPos(chrono.ChVector3d(1, -3, 0))
body_2.SetMass(1.0)
body_2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
body_2.EnableCollision(False)
sys.AddBody(body_2)

box_2 = chrono.ChVisualShapeBox(1, 1, 1)
box_2.SetColor(chrono.ChColor(0.0, 0.0, 0.6))
body_2.AddVisualShape(box_2)

# ------------------------------------------------------------------
# 6) Spring 1: direct-coefficient ChLinkTSDA between body_1 and ground
# ------------------------------------------------------------------
spring_1 = chrono.ChLinkTSDA()
# Initialize anchors in local coords: body at (0,0,0), ground at (-1,0,0)
spring_1.Initialize(body_1, ground,
                    True,
                    chrono.ChVector3d(0, 0, 0),
                    chrono.ChVector3d(-1, 0, 0))
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_1)

# Optional spring visualization
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

# ------------------------------------------------------------------
# 7) Spring 2: uses the custom MySpringForce functor
# ------------------------------------------------------------------
spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_2, ground,
                    True,
                    chrono.ChVector3d(0, 0, 0),
                    chrono.ChVector3d(1, 0, 0))
spring_2.SetRestLength(rest_length)
# Instead of direct SetSpringCoefficient/SetDampingCoefficient, assign functor
spring_2.SetForceFunctor(MySpringForce(spring_coef, damping_coef))
sys.AddLink(spring_2)

spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

# ------------------------------------------------------------------
# 8) Irrlicht visualization loop
# ------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ChLinkTSDA with Custom Force Functor')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 6))
vis.AddTypicalLights()

step_size = 1e-3
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(step_size)