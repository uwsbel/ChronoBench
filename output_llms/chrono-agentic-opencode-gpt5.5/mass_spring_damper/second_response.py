"""Two-body mass-spring-damper PyChrono demo using an NSC system.

The model contains a fixed ground body with two visible anchor spheres, two
mirrored moving spherical masses, and two translational spring-damper links.
The first link uses direct spring and damping coefficients, while the second
link uses a custom ForceFunctor with the same coefficients to compute force.
Both masses should oscillate from an initial stretch under gravity.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants ===
TIME_STEP = 1.0e-3
SIM_END = 6.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

MASS = 1.0
INERTIA = 0.1
BODY_RADIUS = 0.15
ANCHOR_RADIUS = 0.08
SPRING_COEF = 50.0
DAMPING_COEF = 1.0
REST_LENGTH = 1.0

ANCHOR_1 = chrono.ChVector3d(-1.0, 0.0, 0.0)
ANCHOR_2 = chrono.ChVector3d(1.0, 0.0, 0.0)
BODY_1_START = chrono.ChVector3d(-1.0, -1.45, 0.0)
BODY_2_START = chrono.ChVector3d(1.0, -1.45, 0.0)


# === Custom force functor ===
class MySpringForce(chrono.ForceFunctor):
    """Linear spring-damper force used by the second TSDA link."""

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
sys.AddBody(ground)

sph_1 = chrono.ChVisualShapeSphere(ANCHOR_RADIUS)
sph_1.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
ground.AddVisualShape(sph_1, chrono.ChFramed(ANCHOR_1, chrono.QUNIT))

sph_2 = chrono.ChVisualShapeSphere(ANCHOR_RADIUS)
sph_2.SetColor(chrono.ChColor(0.2, 0.2, 0.8))
ground.AddVisualShape(sph_2, chrono.ChFramed(ANCHOR_2, chrono.QUNIT))

body_1 = chrono.ChBody()
body_1.SetMass(MASS)
body_1.SetInertiaXX(chrono.ChVector3d(INERTIA, INERTIA, INERTIA))
body_1.SetPos(BODY_1_START)
body_1.EnableCollision(False)
shape_1 = chrono.ChVisualShapeSphere(BODY_RADIUS)
shape_1.SetColor(chrono.ChColor(0.9, 0.4, 0.1))
body_1.AddVisualShape(shape_1)
sys.AddBody(body_1)

body_2 = chrono.ChBody()
body_2.SetMass(MASS)
body_2.SetInertiaXX(chrono.ChVector3d(INERTIA, INERTIA, INERTIA))
body_2.SetPos(BODY_2_START)
body_2.EnableCollision(False)
shape_2 = chrono.ChVisualShapeSphere(BODY_RADIUS)
shape_2.SetColor(chrono.ChColor(0.1, 0.5, 0.9))
body_2.AddVisualShape(shape_2)
sys.AddBody(body_2)


# === Joints / springs ===
spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground, False, BODY_1_START, ANCHOR_1)
spring_1.SetRestLength(REST_LENGTH)
spring_1.SetSpringCoefficient(SPRING_COEF)
spring_1.SetDampingCoefficient(DAMPING_COEF)
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 12))
sys.AddLink(spring_1)

spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_2, ground, False, BODY_2_START, ANCHOR_2)
spring_2.SetRestLength(REST_LENGTH)
custom_force = MySpringForce(SPRING_COEF, DAMPING_COEF)
spring_2.RegisterForceFunctor(custom_force)
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 12))
sys.AddLink(spring_2)


# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Mass spring damper with custom force functor")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, 1.0, 4.0), chrono.ChVector3d(0.0, -0.75, 0.0))
vis.AddTypicalLights()
grid_frame = chrono.ChCoordsysd(chrono.ChVector3d(0.0, -1.8, 0.0), chrono.QuatFromAngleX(math.pi / 2.0))
vis.AddGrid(0.25, 0.25, 12, 12, grid_frame, chrono.ChColor(0.35, 0.35, 0.35))


# === Main loop ===

try:

    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (OSError, IOError) as exc:
    print(f"Output file or directory error: {exc}")
    raise
except (RuntimeError, ValueError) as exc:
    print(f"Simulation runtime error: {exc}")
    raise
finally:
    print("Simulation resources finalized.")
