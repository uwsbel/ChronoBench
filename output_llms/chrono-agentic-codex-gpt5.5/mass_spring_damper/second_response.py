"""Two independent PyChrono mass-spring-damper oscillators.

This NSC multi-body simulation creates a fixed ground body with two visible
anchor markers, two moving spherical masses, and two translational spring-damper
links. The left spring uses direct stiffness and damping coefficients; the right
spring uses a custom ForceFunctor with the same physical law.
"""

import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants ===
TIME_STEP = 1.0e-3
SIM_END = 5.0
SPRING_COEF = 50.0
DAMPING_COEF = 1.0
REST_LENGTH = 1.0
BODY_MASS = 1.0
BODY_RADIUS = 0.12
GROUND_MARKER_RADIUS = 0.08
ANCHOR_X = 1.0
INITIAL_DROP = -1.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once


# === Custom spring force ===
class MySpringForce(chrono.ForceFunctor):
    """Linear spring-damper force law for ChLinkTSDA."""

    def __init__(self, spring_coef, damping_coef):
        chrono.ForceFunctor.__init__(self)
        self.spring_coef = spring_coef
        self.damping_coef = damping_coef

    def evaluate(self, time, rest_length, length, vel, link):
        return -self.spring_coef * (length - rest_length) - self.damping_coef * vel


# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))


# === Bodies ===
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0.0, 0.0, 0.0))
ground.EnableCollision(False)

sph_1 = chrono.ChVisualShapeSphere(GROUND_MARKER_RADIUS)
sph_1.SetColor(chrono.ChColor(0.15, 0.2, 0.8))
ground.AddVisualShape(sph_1, chrono.ChFramed(chrono.ChVector3d(-ANCHOR_X, 0.0, 0.0)))

sph_2 = chrono.ChVisualShapeSphere(GROUND_MARKER_RADIUS)
sph_2.SetColor(chrono.ChColor(0.15, 0.2, 0.8))
ground.AddVisualShape(sph_2, chrono.ChFramed(chrono.ChVector3d(ANCHOR_X, 0.0, 0.0)))
sys.AddBody(ground)

body_1 = chrono.ChBody()
body_1.SetMass(BODY_MASS)
body_1.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
body_1.SetPos(chrono.ChVector3d(-ANCHOR_X, INITIAL_DROP, 0.0))
body_1.EnableCollision(False)
body_1_shape = chrono.ChVisualShapeSphere(BODY_RADIUS)
body_1_shape.SetColor(chrono.ChColor(0.85, 0.25, 0.15))
body_1.AddVisualShape(body_1_shape)
sys.AddBody(body_1)

body_2 = chrono.ChBody()
body_2.SetMass(BODY_MASS)
body_2.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
body_2.SetPos(chrono.ChVector3d(ANCHOR_X, INITIAL_DROP, 0.0))
body_2.EnableCollision(False)
body_2_shape = chrono.ChVisualShapeSphere(BODY_RADIUS)
body_2_shape.SetColor(chrono.ChColor(0.15, 0.65, 0.35))
body_2.AddVisualShape(body_2_shape)
sys.AddBody(body_2)


# === Springs ===
spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(
    body_1,
    ground,
    True,
    chrono.ChVector3d(0.0, 0.0, 0.0),
    chrono.ChVector3d(-ANCHOR_X, 0.0, 0.0),
)
spring_1.SetRestLength(REST_LENGTH)
spring_1.SetSpringCoefficient(SPRING_COEF)
spring_1.SetDampingCoefficient(DAMPING_COEF)
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 12))
sys.AddLink(spring_1)

spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(
    body_2,
    ground,
    True,
    chrono.ChVector3d(0.0, 0.0, 0.0),
    chrono.ChVector3d(ANCHOR_X, 0.0, 0.0),
)
spring_2.SetRestLength(REST_LENGTH)
force_functor = MySpringForce(SPRING_COEF, DAMPING_COEF)  # cache: retained for link lifetime
spring_2.RegisterForceFunctor(force_functor)
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 12))
sys.AddLink(spring_2)


# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Mass Spring Damper")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, -3.2, 3.5), chrono.ChVector3d(0.0, -0.7, 0.0))
vis.AddTypicalLights()
vis.AddGrid(0.25, 0.25, 12, 12, chrono.ChCoordsysd(), chrono.ChColor(0.45, 0.45, 0.45))


# === Main loop ===


try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(TIME_STEP)
except (OSError, IOError) as exc:  # file creation or write failure
    traceback.print_exc()
    raise
except RuntimeError as exc:  # Chrono runtime or Irrlicht device failure
    traceback.print_exc()
    raise
finally:
    pass


# === Post-processing ===
