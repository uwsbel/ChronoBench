"""
Mass-Spring-Damper simulation using ChLinkTSDA.

Demonstrates a single mass attached to ground via a translational spring-damper
(ChLinkTSDA) with default linear parameters. The mass is given an initial
displacement to exhibit free oscillation. Visualization via Irrlicht with a
visible spring link and an anchor post.
"""

import os
import csv
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Simulation parameters ===
time_step = 1e-3          # physics time step [s]
sim_end = 20.0            # simulation duration [s]
render_fps = 50.0         # rendered frames per second
render_every = max(1, round(1.0 / (render_fps * time_step)))

# Mass and geometry
mass_value = 1.0          # kg
mass_radius = 0.15        # m (sphere radius)
anchor_radius = 0.05      # m (small anchor post sphere)

# Spring-damper parameters (default ChLinkTSDA behaviour)
spring_k = 100.0          # spring stiffness [N/m]
damping_c = 5.0           # damping coefficient [Ns/m]
rest_length = 1.0         # natural length [m] — anchor at y=-1, mass at y=0

# Vertical offset: anchor fixed at y = -1 (bottom), mass starts at y = 0 (above)
anchor_pos = chrono.ChVector3d(0, -1, 0)
mass_initial_pos = chrono.ChVector3d(0, 0, 0)

# === System ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Bodies ===
# Anchor post (fixed to ground)
anchor = chrono.ChBody()
anchor.SetFixed(True)
anchor.SetMass(1e-6)
anchor.SetInertiaXX(chrono.ChVector3d(1e-9, 1e-9, 1e-9))
anchor.SetPos(anchor_pos)
anchor_shape = chrono.ChVisualShapeSphere(anchor_radius)
anchor_shape.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
anchor.AddVisualShape(anchor_shape)
sys.AddBody(anchor)

# Mass sphere
mass_body = chrono.ChBody()
mass_body.SetMass(mass_value)
mass_body.SetInertiaXX(chrono.ChVector3d(
    (2.0 / 5.0) * mass_value * mass_radius ** 2,
    (2.0 / 5.0) * mass_value * mass_radius ** 2,
    (2.0 / 5.0) * mass_value * mass_radius ** 2
))
mass_body.SetPos(mass_initial_pos)
mass_shape = chrono.ChVisualShapeSphere(mass_radius)
mass_shape.SetColor(chrono.ChColor(0.2, 0.6, 0.8))
mass_body.AddVisualShape(mass_shape)
sys.AddBody(mass_body)

# === Spring-Damper (ChLinkTSDA) ===
spring_link = chrono.ChLinkTSDA()
spring_link.Initialize(
    mass_body,
    anchor,
    True,
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(0, 0, 0)
)
spring_link.SetRestLength(rest_length)
spring_link.SetSpringCoefficient(spring_k)
spring_link.SetDampingCoefficient(damping_c)
spring_link.AddVisualShape(
    chrono.ChVisualShapeSpring(0.1, 65, 8)
)
sys.AddLink(spring_link)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Mass-Spring-Damper")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2.5, -1.5, 2.0), chrono.ChVector3d(0, -0.5, 0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 30, 30,
            chrono.ChCoordsysd(chrono.ChVector3d(0, -1.0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Review-only setup ===
REC = bool(os.environ.get("SIMBENCH_RECORD"))       # reviewed: controls capture


# === Main loop ===
frame = 0
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # review-only: capture Irrlicht frame

    for _ in range(render_every):
        t = sys.GetChTime()
        spring_len = spring_link.GetLength()
        spring_vel = spring_link.GetVelocity()
        spring_f   = spring_link.GetForce()
        mass_y     = mass_body.GetPos().y

        # review-only: log CSV every physics step

        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break

# review-only: assemble video + plot table + cleanup
