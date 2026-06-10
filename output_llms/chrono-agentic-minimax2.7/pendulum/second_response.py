"""
Single pendulum simulation on a spherical joint.

Modifications from base pendulum:
- Spherical joint (ChLinkLockSpherical) replacing revolute
- Joint visualized as sphere with radius 2
- Pendulum mass 2 kg, inertia (0.4, 1.5, 1.5)
- Cylinder visual: radius 0.1, height 1.5
- Initial angular velocity applied
- Moon gravity: (0, -1.62, 0)
"""

import os
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Physical constants ===
PENDULUM_MASS = 2.0  # kg
PENDULUM_INERTIA = chrono.ChVector3d(0.4, 1.5, 1.5)  # kg*m^2
PENDULUM_LENGTH = 1.5  # m (cylinder height)
PENDULUM_RADIUS = 0.1  # m (cylinder radius)
JOINT_SPHERE_RADIUS = 2.0  # m (visualization of spherical joint)
GRAVITY_MOON = chrono.ChVector3d(0, -1.62, 0)  # m/s^2 (moon gravity)
INIT_ANG_VEL = chrono.ChVector3d(0, 0, 2.0)  # rad/s initial angular velocity

TIME_STEP = 1e-3  # s
SIM_END = 10.0  # s
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))

# === System ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(GRAVITY_MOON)

# === Bodies ===
# Ground (fixed anchor)
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0, 0, 0))
sys.AddBody(ground)

# Anchor sphere visual (the joint visualization)
anchor_sphere = chrono.ChVisualShapeSphere(JOINT_SPHERE_RADIUS)
anchor_sphere.SetColor(chrono.ChColor(0.6, 0.6, 0.6))
ground.AddVisualShape(anchor_sphere, chrono.ChFramed())

# Pendulum body (link)
# Position: hanging down from origin at (0, 0, -PENDULUM_LENGTH/2)
pendulum = chrono.ChBody()
pendulum.SetMass(PENDULUM_MASS)
pendulum.SetInertiaXX(PENDULUM_INERTIA)
pendulum.SetPos(chrono.ChVector3d(0, 0, -PENDULUM_LENGTH / 2.0))
# Initial angular velocity
pendulum.SetAngVelParent(INIT_ANG_VEL)
sys.AddBody(pendulum)

# Cylinder visual for pendulum arm
# Cylinder axis is body-local Z by default; for a hanging pendulum we want it along body-local X
# so we rotate: QuatFromAngleY(PI/2) maps Z -> X
cyl_vis = chrono.ChVisualShapeCylinder(PENDULUM_RADIUS, PENDULUM_LENGTH)
cyl_vis.SetColor(chrono.ChColor(0.2, 0.5, 0.8))
pendulum.AddVisualShape(cyl_vis, chrono.ChFramed(
    chrono.ChVector3d(0, 0, 0),
    chrono.QuatFromAngleY(chrono.CH_PI_2)
))

# End mass sphere at the bottom of the pendulum
end_sphere = chrono.ChVisualShapeSphere(0.15)
end_sphere.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
pendulum.AddVisualShape(end_sphere, chrono.ChFramed(
    chrono.ChVector3d(0, 0, -PENDULUM_LENGTH / 2.0),
    chrono.QUNIT
))

# === Joint: spherical joint (connects pendulum to ground at origin) ===
spherical_joint = chrono.ChLinkLockSpherical()
spherical_joint.Initialize(
    pendulum, ground,
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0))  # connection point in body-local coords
)
sys.AddLink(spherical_joint)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Pendulum on Spherical Joint - Moon Gravity")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(4, -4, 3), chrono.ChVector3d(0, 0, -0.75))
vis.AddTypicalLights()
vis.AddGrid(1.0, 1.0, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === CSV logging (review-only) ===


csv_path = "simulation_data.csv"
csv_file = None
csv_writer = None

# === Main loop ===
frame = 0
try:
        csv_file = open(csv_path, "w", newline="")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["time", "pos_x", "pos_y", "pos_z", "angvel_x", "angvel_y", "angvel_z"])

    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
            vis.WriteImageToFile(rec.frame_path(irr_dir, frame))
            frame += 1
        for _ in range(RENDER_EVERY):
            t = sys.GetChTime()
            p = pendulum.GetPos()
            av = pendulum.GetAngVelParent()
                csv_writer.writerow([t, p.x, p.y, p.z, av.x, av.y, av.z])
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
finally:
    if csv_file:
        csv_file.close()
