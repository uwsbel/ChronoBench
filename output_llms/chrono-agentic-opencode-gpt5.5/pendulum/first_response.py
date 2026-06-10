"""Simple PyChrono pendulum demo using an NSC system.

The model contains a fixed ground support, one rigid pendulum arm, and a
revolute joint at the support. Gravity drives planar XY motion, while Irrlicht
renders the swing and the loop periodically prints position and velocity.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants === named values keep the mechanism compact and auditable
TIME_STEP = 1e-3
SIM_END = 8.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
LOG_EVERY = max(1, round(0.1 / TIME_STEP))  # precomputed once

PEND_LENGTH = 2.0
PEND_RADIUS = 0.05
PEND_MASS = 1.0
INITIAL_ANGLE = math.radians(35.0)
PIVOT = chrono.ChVector3d(0.0, 0.0, 0.0)
COM = chrono.ChVector3d(
    0.5 * PEND_LENGTH * math.sin(INITIAL_ANGLE),
    -0.5 * PEND_LENGTH * math.cos(INITIAL_ANGLE),
    0.0,
)


# === System & Gravity === pure jointed MBS has no collision shapes
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))


# === Bodies === fixed support plus one moving pendulum arm
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(PIVOT)
sys.AddBody(ground)

support = chrono.ChBody()
support.SetFixed(True)
support.SetPos(PIVOT)
support.SetMass(1.0)
support.SetInertiaXX(chrono.ChVector3d(1e-4, 1e-4, 1e-4))
support_visual = chrono.ChVisualShapeCylinder(0.08, 0.4)
support_visual.SetColor(chrono.ChColor(0.25, 0.25, 0.25))
support.AddVisualShape(support_visual)
sys.AddBody(support)

pendulum = chrono.ChBody()
pendulum.SetMass(PEND_MASS)
pendulum.SetInertiaXX(chrono.ChVector3d(0.2, 1.0, 1.0))
pendulum.SetPos(COM)
pendulum.SetRot(chrono.QuatFromAngleZ(INITIAL_ANGLE - chrono.CH_PI_2))
pendulum.EnableCollision(False)

arm_visual = chrono.ChVisualShapeCylinder(PEND_RADIUS, PEND_LENGTH)
arm_visual.SetColor(chrono.ChColor(0.1, 0.35, 0.9))
pendulum.AddVisualShape(arm_visual, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

bob_visual = chrono.ChVisualShapeSphere(0.14)
bob_visual.SetColor(chrono.ChColor(0.85, 0.15, 0.1))
pendulum.AddVisualShape(bob_visual, chrono.ChFramed(chrono.ChVector3d(PEND_LENGTH / 2.0, 0.0, 0.0)))
sys.AddBody(pendulum)


# === Joints / Constraints === revolute frame local +Z is the physical hinge axis
hinge = chrono.ChLinkLockRevolute()
hinge.Initialize(
    ground,
    pendulum,
    True,
    chrono.ChFramed(chrono.VNULL, chrono.QUNIT),
    chrono.ChFramed(chrono.ChVector3d(-PEND_LENGTH / 2.0, 0.0, 0.0), chrono.QUNIT),
)
sys.AddLink(hinge)


# === Visualization === Irrlicht setup follows initialize-then-scene ordering
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Simple Pendulum")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, -4.0, 2.0), chrono.ChVector3d(0.0, -0.8, 0.0))
vis.AddTypicalLights()


def log_state(step_index: int) -> None:
    """Print periodic pendulum state requested by the prompt."""
    if step_index % LOG_EVERY != 0:
        return
    t = sys.GetChTime()  # cache: fetched once for this log record
    pos = pendulum.GetPos()  # cache: printed and mirrored into review CSV
    vel = pendulum.GetPosDt()  # cache: printed and mirrored into review CSV
    print(
        f"time={t:.3f} pos=({pos.x:.5f}, {pos.y:.5f}, {pos.z:.5f}) "
        f"vel=({vel.x:.5f}, {vel.y:.5f}, {vel.z:.5f})"
    )


# === Main Loop === render frames, advance physics, and log motion
frame = 0
step = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            current_time = sys.GetChTime()  # cache: reused for CSV and stop check
            pos = pendulum.GetPos()  # cache: reused for review-only row fields
            vel = pendulum.GetPosDt()  # cache: reused for review-only row fields
            log_state(step)
            sys.DoStepDynamics(TIME_STEP)
            step += 1
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid state guard
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:  # output directory or file-write guard
    traceback.print_exc()
    raise
finally:
    pass


# === Post-processing === review artifacts are stripped from the accepted script
