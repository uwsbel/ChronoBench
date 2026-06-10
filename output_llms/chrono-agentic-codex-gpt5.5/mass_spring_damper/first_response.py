"""Single mass-spring-damper demo using a PyChrono NSC system.

A fixed ground body anchors one end of a ChLinkTSDA, and a single moving mass is
connected at the other end. The mass starts stretched beyond the spring rest
length, so the default linear spring-damper force pulls it into damped
oscillation while Irrlicht shows the mass, anchor, and spring.
"""

import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants === named values keep the spring setup clear and bounded
TIME_STEP = 1.0e-3
SIM_END = 6.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

MASS_VALUE = 1.0
MASS_INERTIA = 0.02
MASS_SIZE = 0.35
ANCHOR_WIDTH = 0.12
ANCHOR_HEIGHT = 0.9
ANCHOR_DEPTH = 0.9
REST_LENGTH = 1.0
SPRING_K = 10.0
DAMPING_C = 0.15
INITIAL_STRETCH = 1.0

GROUND_POS = chrono.ChVector3d(0.0, 0.0, 0.0)
MASS_POS = chrono.ChVector3d(REST_LENGTH + INITIAL_STRETCH, 0.0, 0.0)
SPRING_ATTACH_GROUND = chrono.ChVector3d(0.0, 0.0, 0.0)
SPRING_ATTACH_MASS = chrono.ChVector3d(0.0, 0.0, 0.0)


# === System & gravity === no contact, so no collision system is installed
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, 0.0))


# === Bodies === fixed anchor and one moving mass with visible shapes
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(GROUND_POS)
ground_shape = chrono.ChVisualShapeBox(ANCHOR_WIDTH, ANCHOR_HEIGHT, ANCHOR_DEPTH)
ground_shape.SetColor(chrono.ChColor(0.25, 0.25, 0.25))
ground.AddVisualShape(ground_shape)
sys.AddBody(ground)

mass = chrono.ChBody()
mass.SetMass(MASS_VALUE)
mass.SetInertiaXX(chrono.ChVector3d(MASS_INERTIA, MASS_INERTIA, MASS_INERTIA))
mass.SetPos(MASS_POS)
mass.SetPosDt(chrono.ChVector3d(0.0, 0.0, 0.0))
mass_shape = chrono.ChVisualShapeBox(MASS_SIZE, MASS_SIZE, MASS_SIZE)
mass_shape.SetColor(chrono.ChColor(0.1, 0.35, 0.85))
mass.AddVisualShape(mass_shape)
sys.AddBody(mass)


# === Joints / constraints === TSDA connects the mass to the fixed anchor
spring = chrono.ChLinkTSDA()
spring.Initialize(ground, mass, True, SPRING_ATTACH_GROUND, SPRING_ATTACH_MASS)
spring.SetRestLength(REST_LENGTH)
spring.SetSpringCoefficient(SPRING_K)
spring.SetDampingCoefficient(DAMPING_C)
spring.AddVisualShape(chrono.ChVisualShapeSpring(0.08, 80, 12))
sys.AddLink(spring)


# === Visualization === Irrlicht scene shows the spring and the moving mass
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Mass-Spring-Damper")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2.7, 1.6, 2.6), chrono.ChVector3d(0.9, 0.0, 0.0))
vis.AddTypicalLights()


# === Main loop === render at video cadence while stepping the spring dynamics
mass_body = mass  # cache: repeated state reads use one local handle
spring_link = spring  # cache: repeated spring reads use one local handle


frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sim_time = sys.GetChTime()  # cache: read once for logging and bounds
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid numeric state
    traceback.print_exc()
    raise
finally:
    pass
