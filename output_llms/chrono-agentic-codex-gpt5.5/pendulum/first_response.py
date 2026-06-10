"""Simple PyChrono pendulum demo.

This script builds a pure jointed MBS model with a fixed ground body and one
rigid pendulum body in an NSC system under Y-up gravity.  The pendulum is
attached to ground by a revolute joint about world Z, rendered with Irrlicht,
and periodically prints position and velocity as it swings.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants === Direct values keep this close to the canonical pendulum demo
TIME_STEP = 0.001
SIM_END = 8.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
LOG_INTERVAL = 0.1

PENDULUM_MASS = 1.0
PENDULUM_LENGTH = 2.0
PENDULUM_RADIUS = 0.06
PENDULUM_ANGLE = math.radians(35.0)
PENDULUM_INERTIA = chrono.ChVector3d(0.2, 1.0, 1.0)

PIVOT = chrono.ChVector3d(0.0, 0.0, 0.0)
PENDULUM_POS = chrono.ChVector3d(
    0.5 * PENDULUM_LENGTH * math.sin(PENDULUM_ANGLE),
    -0.5 * PENDULUM_LENGTH * math.cos(PENDULUM_ANGLE),
    0.0,
)
PENDULUM_ROT = chrono.QuatFromAngleZ(PENDULUM_ANGLE - chrono.CH_PI_2)


# === System & gravity === NSC is sufficient because this jointed model has no contact
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))


# === Bodies === Fixed truss plus a manually oriented cylindrical pendulum arm
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(PIVOT)
sys.AddBody(ground)

pivot_marker = chrono.ChVisualShapeSphere(0.09)
pivot_marker.SetColor(chrono.ChColor(0.95, 0.1, 0.05))
ground.AddVisualShape(pivot_marker, chrono.ChFramed(PIVOT, chrono.QUNIT))

support_shape = chrono.ChVisualShapeCylinder(0.035, 1.0)
support_shape.SetColor(chrono.ChColor(0.1, 0.1, 0.1))
ground.AddVisualShape(support_shape, chrono.ChFramed(PIVOT, chrono.QUNIT))

base_shape = chrono.ChVisualShapeBox(0.5, 0.05, 1.0)
base_shape.SetColor(chrono.ChColor(0.15, 0.15, 0.15))
ground.AddVisualShape(base_shape, chrono.ChFramed(chrono.ChVector3d(0.0, 0.08, 0.0), chrono.QUNIT))

pendulum = chrono.ChBody()
pendulum.SetMass(PENDULUM_MASS)
pendulum.SetInertiaXX(PENDULUM_INERTIA)
pendulum.SetPos(PENDULUM_POS)
pendulum.SetRot(PENDULUM_ROT)
pendulum.EnableCollision(False)
sys.AddBody(pendulum)

arm_shape = chrono.ChVisualShapeCylinder(PENDULUM_RADIUS, PENDULUM_LENGTH)
arm_shape.SetColor(chrono.ChColor(0.2, 0.45, 0.9))
pendulum.AddVisualShape(
    arm_shape,
    chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)),
)


# === Joints / constraints === Revolute local +Z aligns with world Z for XY swing
revolute = chrono.ChLinkLockRevolute()
revolute.Initialize(pendulum, ground, chrono.ChFramed(PIVOT, chrono.QUNIT))
sys.AddLink(revolute)


# === Visualization === Irrlicht window is initialized before scene elements
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono Simple Pendulum")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, -4.5, 1.4), chrono.ChVector3d(0.0, -0.6, 0.0))
vis.AddTypicalLights()


# === Cached handles === Reused state access stays explicit in the hot loop
pendulum_body = pendulum  # cache: reused for periodic logs


# === Main loop === Render at frame cadence, integrate the pendulum every step
frame = 0
next_log_time = 0.0

try:
        while vis.Run() and sys.GetChTime() < SIM_END:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            for _ in range(RENDER_EVERY):
                sim_time = sys.GetChTime()  # cache: one time query drives log cadence
                pos = pendulum_body.GetPos()  # cache: body state for console output
                vel = pendulum_body.GetPosDt()  # cache: body state for console output
                if sim_time >= next_log_time:
                    print(
                        f"t={sim_time:.3f} pos=({pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f}) "
                        f"vel=({vel.x:.4f}, {vel.y:.4f}, {vel.z:.4f})"
                    )
                    next_log_time += LOG_INTERVAL
                sys.DoStepDynamics(TIME_STEP)
                if sys.GetChTime() >= SIM_END:
                    break
except (RuntimeError, ValueError) as exc:  # Chrono solver/runtime state failures
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:  # disk or permission failures during review recording
    traceback.print_exc()
    raise
finally:
    print(f"Simulation finished at t={sys.GetChTime():.3f}")
