"""Double pendulum in a PyChrono NSC multi-body system.

The simulation builds a fixed truss and two rigid pendulum arms connected by
revolute joints in the XY plane. Gravity acts in -Y, so both independently
hinged arms swing as a coupled double-pendulum mechanism.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants: define the mechanism and run cadence ===
TIME_STEP = 1.0e-3
SIM_END = 8.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

ARM1_LENGTH = 1.2
ARM2_LENGTH = 1.0
ARM_RADIUS = 0.045
ARM1_MASS = 1.0
ARM2_MASS = 0.8
PIVOT_RADIUS = 0.07

ARM1_ANGLE = -0.82
ARM2_ANGLE = -1.85
PIVOT_POS = chrono.ChVector3d(0.0, 0.0, 0.0)


def make_arm(name, length, radius, mass, angle, center):
    """Create one pendulum arm with local X aligned to its hinge-to-tip axis."""
    arm = chrono.ChBody()
    arm.SetName(name)
    arm.SetMass(mass)
    arm.SetInertiaXX(chrono.ChVector3d(0.2, 1.0, 1.0))
    arm.SetPos(center)
    arm.SetRot(chrono.QuatFromAngleZ(angle))
    arm.EnableCollision(False)

    cyl = chrono.ChVisualShapeCylinder(radius, length)
    cyl.SetColor(chrono.ChColor(0.1, 0.35, 0.85))
    arm.AddVisualShape(cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

    bob = chrono.ChVisualShapeSphere(radius * 1.8)
    bob.SetColor(chrono.ChColor(0.85, 0.25, 0.1))
    arm.AddVisualShape(bob, chrono.ChFramed(chrono.ChVector3d(length / 2.0, 0.0, 0.0), chrono.QUNIT))
    return arm


def endpoint_world(body, local_x):
    """Return a body-local X-axis endpoint in world coordinates."""
    pos = body.GetPos()  # cache: reused with the rotation below
    rot = body.GetRot()  # cache: reused to transform the endpoint
    return pos + rot.RotateBack(chrono.ChVector3d(local_x, 0.0, 0.0))


# === System & gravity: pure jointed MBS without contact ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))


# === Bodies: fixed support plus two rigid pendulum arms ===
ground = chrono.ChBody()
ground.SetName("fixed_truss")
ground.SetFixed(True)
sys.AddBody(ground)

arm1_dir = chrono.ChVector3d(math.cos(ARM1_ANGLE), math.sin(ARM1_ANGLE), 0.0)
arm1_center = PIVOT_POS + arm1_dir * (ARM1_LENGTH / 2.0)
arm1 = make_arm("upper_pendulum_arm", ARM1_LENGTH, ARM_RADIUS, ARM1_MASS, ARM1_ANGLE, arm1_center)
sys.AddBody(arm1)

arm1_tip = endpoint_world(arm1, ARM1_LENGTH / 2.0)
arm2_dir = chrono.ChVector3d(math.cos(ARM2_ANGLE), math.sin(ARM2_ANGLE), 0.0)
arm2_center = arm1_tip + arm2_dir * (ARM2_LENGTH / 2.0)
arm2 = make_arm("lower_pendulum_arm", ARM2_LENGTH, ARM_RADIUS, ARM2_MASS, ARM2_ANGLE, arm2_center)
arm2.SetAngVelParent(chrono.ChVector3d(0.0, 0.0, 1.0))
sys.AddBody(arm2)

support = chrono.ChVisualShapeSphere(PIVOT_RADIUS)
support.SetColor(chrono.ChColor(0.1, 0.1, 0.1))
ground.AddVisualShape(support, chrono.ChFramed(PIVOT_POS, chrono.QUNIT))

joint_marker = chrono.ChVisualShapeSphere(PIVOT_RADIUS * 0.85)
joint_marker.SetColor(chrono.ChColor(0.05, 0.05, 0.05))
arm1.AddVisualShape(joint_marker, chrono.ChFramed(chrono.ChVector3d(ARM1_LENGTH / 2.0, 0.0, 0.0), chrono.QUNIT))


# === Joints / constraints: two revolute hinges about world Z ===
ground_hinge = chrono.ChLinkLockRevolute()
ground_hinge.Initialize(
    ground,
    arm1,
    True,
    chrono.ChFramed(PIVOT_POS, chrono.QUNIT),
    chrono.ChFramed(chrono.ChVector3d(-ARM1_LENGTH / 2.0, 0.0, 0.0), chrono.QUNIT),
)
sys.AddLink(ground_hinge)

inter_arm_hinge = chrono.ChLinkLockRevolute()
inter_arm_hinge.Initialize(
    arm1,
    arm2,
    True,
    chrono.ChFramed(chrono.ChVector3d(ARM1_LENGTH / 2.0, 0.0, 0.0), chrono.QUNIT),
    chrono.ChFramed(chrono.ChVector3d(-ARM2_LENGTH / 2.0, 0.0, 0.0), chrono.QUNIT),
)
sys.AddLink(inter_arm_hinge)


# === Visualization: Irrlicht window built unconditionally ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Double Pendulum")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.0, -4.0, 2.0), chrono.ChVector3d(0.2, -0.8, 0.0))
vis.AddTypicalLights()


# === Main loop: render, record review data when requested, and step dynamics ===

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:
    traceback.print_exc()
    raise
finally:
    pass
