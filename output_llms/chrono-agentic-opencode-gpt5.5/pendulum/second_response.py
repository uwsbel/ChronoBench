"""Moon-gravity spherical pendulum using a PyChrono NSC multi-body system.

The model contains a fixed pivot body visualized as a radius-2 sphere, one
dynamic cylindrical pendulum body, and a spherical joint at the pendulum top.
The pendulum has mass 2 kg, inertia (0.4, 1.5, 1.5), length 1.5 m, and an
initial angular velocity so it swings smoothly under lunar gravity.
"""

import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants === direct prompt values and run controls.
PENDULUM_MASS = 2.0
PENDULUM_INERTIA = chrono.ChVector3d(0.4, 1.5, 1.5)
PENDULUM_RADIUS = 0.1
PENDULUM_LENGTH = 1.5
JOINT_SPHERE_RADIUS = 2.0
INITIAL_ANGULAR_SPEED = 3.0
TIME_STEP = 1e-3
SIM_END = 8.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once


# === System & gravity === pure jointed MBS, so no collision system is required.
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -1.62, 0))


# === Bodies === fixed joint marker and moving cylindrical pendulum.
pivot = chrono.ChBody()
pivot.SetFixed(True)
pivot.SetPos(chrono.ChVector3d(0, 0, 0))
pivot_shape = chrono.ChVisualShapeSphere(JOINT_SPHERE_RADIUS)
pivot_shape.SetColor(chrono.ChColor(0.9, 0.55, 0.1))
pivot_shape.SetOpacity(0.08)
pivot.AddVisualShape(pivot_shape)
sys.AddBody(pivot)

pendulum = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_Y,
    PENDULUM_RADIUS,
    PENDULUM_LENGTH,
    1000.0,
)
pendulum.SetMass(PENDULUM_MASS)
pendulum.SetInertiaXX(PENDULUM_INERTIA)
pendulum.SetPos(chrono.ChVector3d(0, -PENDULUM_LENGTH / 2.0, 0))
pendulum.SetAngVelParent(chrono.ChVector3d(0, 0, INITIAL_ANGULAR_SPEED))
pendulum.GetVisualShape(0).SetColor(chrono.ChColor(0.15, 0.35, 0.95))
pendulum.EnableCollision(False)
sys.AddBody(pendulum)


# === Joints / constraints === spherical joint replaces the revolute hinge.
spherical_joint = chrono.ChLinkLockSpherical()
spherical_joint.Initialize(
    pendulum,
    pivot,
    True,
    chrono.ChFramed(chrono.ChVector3d(0, PENDULUM_LENGTH / 2.0, 0), chrono.QUNIT),
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
)
sys.AddLink(spherical_joint)


# === Visualization === Irrlicht scene initialized before cameras and lights.
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Spherical Pendulum on the Moon")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3.5, 8.0), chrono.ChVector3d(0, -0.55, 0))
vis.AddTypicalLights()
vis.AddGrid(
    0.5,
    0.5,
    20,
    20,
    chrono.ChCoordsysd(chrono.ChVector3d(0, -1.7, 0), chrono.QUNIT),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main loop === render at video cadence and step dynamics in fixed increments.
frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid numeric state
    import traceback
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:  # filesystem errors from renderer output
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
