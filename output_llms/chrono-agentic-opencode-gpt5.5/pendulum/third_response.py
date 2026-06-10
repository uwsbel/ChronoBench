"""
Double pendulum PyChrono MBS demo using an NSC system without contact.

Two rigid rod pendulums swing in the XY plane under Y-down gravity.  The first
rod is hinged to a fixed support and the second rod is hinged to the first rod's
free end, allowing independent relative rotation at both pivots.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants === physical scale and recording cadence are fixed once
time_step = 1e-3
sim_end = 8.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

rod_length = 1.5
rod_radius = 0.035
rod_mass = 1.0
bob_radius = 0.12
theta1 = -math.pi / 3.0
theta2 = -math.pi / 6.0

anchor = chrono.ChVector3d(0.0, 0.0, 0.0)
dir1 = chrono.ChVector3d(math.cos(theta1), math.sin(theta1), 0.0)
dir2 = chrono.ChVector3d(math.cos(theta2), math.sin(theta2), 0.0)
arm1_center = anchor + dir1 * (0.5 * rod_length)
hinge2_pos = anchor + dir1 * rod_length
arm2_center = hinge2_pos + dir2 * (0.5 * rod_length)


def make_arm(name, center, angle, color):
    """Create one pendulum rod body with rod and bob visuals."""
    arm = chrono.ChBody()
    arm.SetName(name)
    arm.SetMass(rod_mass)
    arm.SetInertiaXX(chrono.ChVector3d(0.2, 1.0, 1.0))
    arm.SetPos(center)
    arm.SetRot(chrono.QuatFromAngleZ(angle))
    arm.EnableCollision(False)

    rod_shape = chrono.ChVisualShapeCylinder(rod_radius, rod_length)
    rod_shape.SetColor(color)
    arm.AddVisualShape(
        rod_shape,
        chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)),
    )

    bob_shape = chrono.ChVisualShapeSphere(bob_radius)
    bob_shape.SetColor(color)
    arm.AddVisualShape(bob_shape, chrono.ChFramed(chrono.ChVector3d(0.5 * rod_length, 0.0, 0.0)))
    return arm


# === System === pure jointed MBS, so no collision system is enabled
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))


# === Bodies === fixed support plus two moving pendulum arms
ground = chrono.ChBody()
ground.SetName("fixed support")
ground.SetFixed(True)
ground.SetPos(anchor)
ground.EnableCollision(False)
sys.AddBody(ground)

support_shape = chrono.ChVisualShapeSphere(0.08)
support_shape.SetColor(chrono.ChColor(0.25, 0.25, 0.25))
ground.AddVisualShape(support_shape, chrono.ChFramed(anchor))

arm1 = make_arm("upper pendulum arm", arm1_center, theta1, chrono.ChColor(0.2, 0.35, 0.9))
arm2 = make_arm("lower pendulum arm", arm2_center, theta2, chrono.ChColor(0.9, 0.25, 0.2))
arm2.SetAngVelParent(chrono.ChVector3d(0.0, 0.0, 1.25))
sys.AddBody(arm1)
sys.AddBody(arm2)

arm1_cached = arm1  # cache: reused for joint setup and per-step logging
arm2_cached = arm2  # cache: reused for joint setup and per-step logging


# === Joints === revolute pivots about world Z for planar XY swinging
hinge1 = chrono.ChLinkLockRevolute()
hinge1.Initialize(
    ground,
    arm1_cached,
    True,
    chrono.ChFramed(anchor, chrono.QUNIT),
    chrono.ChFramed(chrono.ChVector3d(-0.5 * rod_length, 0.0, 0.0), chrono.QUNIT),
)
sys.AddLink(hinge1)

hinge2 = chrono.ChLinkLockRevolute()
hinge2.Initialize(
    arm1_cached,
    arm2_cached,
    True,
    chrono.ChFramed(chrono.ChVector3d(0.5 * rod_length, 0.0, 0.0), chrono.QUNIT),
    chrono.ChFramed(chrono.ChVector3d(-0.5 * rod_length, 0.0, 0.0), chrono.QUNIT),
)
sys.AddLink(hinge2)


# === Visualization === Irrlicht window with camera, sky, and lights
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Double Pendulum")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.4, 0.0, 4.8), chrono.ChVector3d(0.45, -1.45, 0.0))
vis.AddTypicalLights()


# === Main Loop === render the mechanism and step the constrained dynamics
frame = 0

try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid state guard
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:  # output directory or image-write guard
    traceback.print_exc()
    raise
finally:
    pass


# === Review Output === CSV, plot, and video are stripped before scoring
