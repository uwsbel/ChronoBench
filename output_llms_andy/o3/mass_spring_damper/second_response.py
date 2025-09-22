#  -----------------------------------------------------------------------------
#  ChLinkTSDA example with a user-defined force functor
#
#  This script shows two identical bodies that are each attached to the ground
#  by a translational spring–damper (TSDA).  The first spring uses the built-in
#  linear‐elastic / linear-damping law, whereas the second one relies on a user
#  force functor (MySpringForce).  Two small spheres are placed on the ground
#  to mark the attachment points of the springs.
#  -----------------------------------------------------------------------------

import pychrono as chrono
import pychrono.irrlicht as chronoirr

# -----------------------------------------------------------------------------
# Simulation parameters
# -----------------------------------------------------------------------------
rest_length   = 1.5          # unloaded length of both springs
spring_coef   = 50.0         # stiffness  [N/m]
damping_coef  = 1.0          # damping    [N·s/m]
time_step     = 1.0e-3       # integrator step size

# -----------------------------------------------------------------------------
# Create the physical system
# -----------------------------------------------------------------------------
sys = chrono.ChSystemNSC()

# (no gravity so the motion is produced only by the springs)
try:                                     # depending on Chrono version
    sys.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, 0))
except AttributeError:
    sys.Set_G_acc(chrono.ChVectorD(0, 0, 0))

# -----------------------------------------------------------------------------
# Ground body (immobile reference)
# -----------------------------------------------------------------------------
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetCollide(False)
sys.AddBody(ground)

# Visual spheres that mark the spring anchor points on the ground
marker_rad = 0.10

sph_1 = chrono.ChVisualShapeSphere(marker_rad)
ground.AddVisualShape(sph_1,
                      chrono.ChFrameD(chrono.ChVectorD(-1, 0, 0)))   # left anchor

sph_2 = chrono.ChVisualShapeSphere(marker_rad)
ground.AddVisualShape(sph_2,
                      chrono.ChFrameD(chrono.ChVectorD( 1, 0, 0)))   # right anchor

# -----------------------------------------------------------------------------
# Helper function: create a suspended cubical body
# -----------------------------------------------------------------------------
def make_body(pos, color):
    body = chrono.ChBody()
    body.SetPos(chrono.ChVectorD(*pos))
    body.SetMass(1.0)
    body.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
    body.SetCollide(False)

    box = chrono.ChVisualShapeBox(1, 1, 1)        # cube of edge length 2
    box.SetColor(color)
    body.AddVisualShape(box)

    sys.AddBody(body)
    return body

# Two identical bodies hanging under the ground
body_1 = make_body(pos = (-1, -3, 0), color = chrono.ChColor(0.6, 0.0, 0.0))
body_2 = make_body(pos = ( 1, -3, 0), color = chrono.ChColor(0.0, 0.0, 0.6))

# -----------------------------------------------------------------------------
# 1) Standard linear spring–damper (body_1  ↔  ground)
# -----------------------------------------------------------------------------
spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground, True,
                    chrono.ChVectorD(0, 0, 0),     # attachment in body_1 frame
                    chrono.ChVectorD(-1, 0, 0))    # attachment in ground frame
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, rest_length))
sys.AddLink(spring_1)

# -----------------------------------------------------------------------------
# 2) Custom force functor ------------------------------------------------------
# -----------------------------------------------------------------------------
class MySpringForce(chrono.ForceFunctor):
    """
    User-defined spring–damper force:
        F = −k (x − x0) − c * v
    """
    def __init__(self, k, c):
        chrono.ForceFunctor.__init__(self)
        self.k = k
        self.c = c

    def __call__(self, time, rest_length, length, vel, link):
        return -self.k * (length - rest_length) - self.c * vel

user_force = MySpringForce(spring_coef, damping_coef)

# -----------------------------------------------------------------------------
#    Spring with custom force (body_2 ↔ ground)
# -----------------------------------------------------------------------------
spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_2, ground, True,
                    chrono.ChVectorD(0, 0, 0),     # attachment in body_2 frame
                    chrono.ChVectorD( 1, 0, 0))    # attachment in ground frame
spring_2.SetRestLength(rest_length)
spring_2.RegisterForceFunctor(user_force)
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, rest_length))
sys.AddLink(spring_2)

# -----------------------------------------------------------------------------
# Irrlicht visualization
# -----------------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("ChLinkTSDA demo with custom force functor")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, 10))
vis.AddTypicalLights()

# -----------------------------------------------------------------------------
# Simulation loop
# -----------------------------------------------------------------------------
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(time_step)