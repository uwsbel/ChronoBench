"""
Mass-Spring-Damper simulation with two bodies and custom force functor.

This SimBench demo models two spring-mass-damper systems connected to ground.
body_1 hangs from spring_1 (direct coefficients). body_2 hangs from spring_2
(using custom MySpringForce functor with spring_coef=50, damping_coef=1).
Both ground spheres are visualized at their anchor points.
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# review-only: sim_recording import and REC flag

# === Named constants ===
time_step = 1e-3
sim_end = 10.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

# Body masses
mass_1 = 1.0
mass_2 = 1.0

# Body sizes (cube)
size_1 = 0.3
size_2 = 0.3

# Spring parameters
spring_coef = 50.0
damping_coef = 1.0
rest_length = 1.0

# Vertical positions
ground_y = 0.0
anchor_y = 2.0
body_1_y = anchor_y - rest_length - size_1 / 2
body_2_y = body_1_y

# Horizontal positions
body_1_x = 0.0
body_2_x = 1.0

# === System ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Custom Force Functor ===
class MySpringForce(chrono.ForceFunctor):
    """Custom spring force functor with spring_coef=50, damping_coef=1."""
    def __init__(self):
        super(MySpringForce, self).__init__()

    def evaluate(self, time, rest_length, length, vel, link):
        # Spring force: F = -k * (length - rest_length) - c * velocity
        force = -50 * (length - rest_length) - 1 * vel
        return force


# === Ground body ===
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0, ground_y, 0))
sys.AddBody(ground)

# Ground sphere visual sph_1 at (0, 0, 0)
sph_1 = chrono.ChVisualShapeSphere(0.05)
sph_1.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
ground.AddVisualShape(sph_1, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))

# Ground sphere visual sph_2 at (1, 0, 0)
sph_2 = chrono.ChVisualShapeSphere(0.05)
sph_2.SetColor(chrono.ChColor(0.2, 0.2, 0.8))
ground.AddVisualShape(sph_2, chrono.ChFramed(chrono.ChVector3d(1, 0, 0), chrono.QUNIT))


# === Body 1 (hanging mass) ===
body_1 = chrono.ChBody()
body_1.SetMass(mass_1)
body_1.SetInertiaXX(chrono.ChVector3d(
    (1.0 / 6.0) * mass_1 * size_1 ** 2,
    (1.0 / 6.0) * mass_1 * size_1 ** 2,
    (1.0 / 6.0) * mass_1 * size_1 ** 2
))
body_1.SetPos(chrono.ChVector3d(body_1_x, body_1_y, 0))
body_1.EnableCollision(False)
sys.AddBody(body_1)

vis_1 = chrono.ChVisualShapeBox(size_1, size_1, size_1)
vis_1.SetColor(chrono.ChColor(0.8, 0.5, 0.2))
body_1.AddVisualShape(vis_1)


# === Body 2 (mirrors body_1) ===
body_2 = chrono.ChBody()
body_2.SetMass(mass_2)
body_2.SetInertiaXX(chrono.ChVector3d(
    (1.0 / 6.0) * mass_2 * size_2 ** 2,
    (1.0 / 6.0) * mass_2 * size_2 ** 2,
    (1.0 / 6.0) * mass_2 * size_2 ** 2
))
body_2.SetPos(chrono.ChVector3d(body_2_x, body_2_y, 0))
body_2.EnableCollision(False)
sys.AddBody(body_2)

vis_2 = chrono.ChVisualShapeBox(size_2, size_2, size_2)
vis_2.SetColor(chrono.ChColor(0.2, 0.5, 0.8))
body_2.AddVisualShape(vis_2)


# === Spring 1: body_1 to ground (direct coefficients) ===
spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(
    body_1, ground, True,
    chrono.ChVector3d(0, -size_1 / 2, 0),  # attachment on body_1 (bottom)
    chrono.ChVector3d(body_1_x, 0, 0)        # anchor on ground
)
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 10, 0.02))
sys.AddLink(spring_1)


# === Spring 2: body_2 to ground (custom force functor) ===
spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(
    body_2, ground, True,
    chrono.ChVector3d(0, -size_2 / 2, 0),  # attachment on body_2 (bottom)
    chrono.ChVector3d(body_2_x, 0, 0)        # anchor on ground
)
spring_2.SetRestLength(rest_length)
my_spring_force = MySpringForce()  # must persist for RegisterForceFunctor
spring_2.RegisterForceFunctor(my_spring_force)
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 10, 0.02))
sys.AddLink(spring_2)


# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Mass-Spring-Damper with Custom Force Functor")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2, -4, 3), chrono.ChVector3d(0.5, 1, 0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# review-only: prepare frame capture

# === CSV logging ===
csv_path = "simulation_data.csv"
csv_file = None
csv_writer = None
# review-only: open CSV before loop
try:
except (OSError, IOError) as exc:
    import traceback; traceback.print_exc()

# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        if REC:
            frame += 1
        for _ in range(render_every):
            t = sys.GetChTime()
            if csv_writer:
                csv_writer.writerow([
                    t,
                    body_1.GetPos().y,
                    body_2.GetPos().y,
                    spring_1.GetForce(),
                    spring_2.GetForce()
                ])
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
finally:
    # review-only: close CSV
    if csv_file:

# review-only: assemble videos and plot
