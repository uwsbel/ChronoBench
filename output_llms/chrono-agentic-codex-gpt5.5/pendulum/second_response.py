"""Single spherical-joint pendulum on lunar gravity.

This PyChrono NSC model contains a fixed truss, a visual joint sphere, and a
2 kg cylindrical pendulum connected by a spherical joint. The pendulum starts
with an angular velocity and swings smoothly under moon gravity.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants ===
TIME_STEP = 0.005
SIM_END = 6.0
PENDULUM_MASS = 2.0
PENDULUM_INERTIA = chrono.ChVector3d(0.4, 1.5, 1.5)
ROD_RADIUS = 0.1
ROD_LENGTH = 1.5
JOINT_SPHERE_RADIUS = 2.0
INITIAL_ANGLE = math.radians(35.0)
INITIAL_ANGULAR_VELOCITY = chrono.ChVector3d(0.0, 0.0, 1.2)
PIVOT = chrono.ChVector3d(0.0, 0.0, 0.0)
ROD_DIRECTION_ANGLE = -chrono.CH_PI_2 + INITIAL_ANGLE
ROD_DIRECTION = chrono.ChVector3d(math.cos(ROD_DIRECTION_ANGLE), math.sin(ROD_DIRECTION_ANGLE), 0.0)
ROD_CENTER = PIVOT + ROD_DIRECTION * (0.5 * ROD_LENGTH)
ROD_ROTATION = chrono.QuatFromAngleZ(ROD_DIRECTION_ANGLE)


# === System & gravity ===
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -1.62, 0))


# === Bodies ===
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetPos(PIVOT)
system.AddBody(truss)

joint_sphere = chrono.ChVisualShapeSphere(JOINT_SPHERE_RADIUS)
joint_sphere.SetColor(chrono.ChColor(0.25, 0.55, 1.0))
joint_sphere.SetOpacity(0.18)
truss.AddVisualShape(joint_sphere, chrono.ChFramed(PIVOT, chrono.QUNIT))

pendulum = chrono.ChBody()
pendulum.SetMass(PENDULUM_MASS)
pendulum.SetInertiaXX(PENDULUM_INERTIA)
pendulum.SetPos(ROD_CENTER)
pendulum.SetRot(ROD_ROTATION)
pendulum.SetAngVelParent(INITIAL_ANGULAR_VELOCITY)
pendulum.EnableCollision(False)
rod_visual = chrono.ChVisualShapeCylinder(ROD_RADIUS, ROD_LENGTH)
rod_visual.SetColor(chrono.ChColor(1.0, 0.15, 0.05))
pendulum.AddVisualShape(rod_visual, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
system.AddBody(pendulum)


# === Joints / constraints ===
spherical = chrono.ChLinkLockSpherical()
spherical.Initialize(
    pendulum,
    truss,
    True,
    chrono.ChFramed(chrono.ChVector3d(-0.5 * ROD_LENGTH, 0.0, 0.0), chrono.QUNIT),
    chrono.ChFramed(PIVOT, chrono.QUNIT),
)
system.AddLink(spherical)


# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Moon Gravity Spherical Pendulum")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(4.5, -7.0, 4.0), chrono.ChVector3d(0.0, -0.6, 0.0))
vis.AddTypicalLights()


# === Main loop ===
pendulum_body = pendulum  # cache: repeated state reads use the same body handle

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(TIME_STEP)
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid state guard
    traceback.print_exc()
    raise
finally:
    final_time = system.GetChTime()


print(f"Simulation completed at t={final_time:.3f} s")
