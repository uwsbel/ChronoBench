"""
Mass-Spring-Damper Chain Simulation (3 Bodies)

Models three bodies (body_1, body_2, body_3) suspended vertically with
springs connecting body_1 to body_2 and body_2 to body_3, all springing
from a fixed ground support. System type: ChSystemNSC (pure jointed MBS
with no collision contact). Each spring-damper element is a ChLinkTSDA
with rest length, spring coefficient, and damping coefficient. The bodies
hang and oscillate under gravity (-Y direction).

Expected behavior: three-body vertical mass-spring chain exhibits coupled
oscillatory motion with each body displacement driven by spring-damper forces.
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants ===
# Physics parameters
time_step = 1e-3        # integration timestep [s]
sim_end   = 10.0        # simulation end time [s]
render_fps = 50.0       # render frames per second

# Body parameters
BODY_MASS    = 1.0      # mass of each body [kg]
BODY_INERTIA = 0.1      # diagonal inertia component [kg·m²]
BODY_RADIUS  = 0.1      # sphere visual radius [m]

# Spring parameters (same for all springs, matching original)
SPRING_K     = 50.0     # spring stiffness [N/m]
DAMPING_C    = 1.0      # damping coefficient [N·s/m]
REST_LENGTH  = 1.0      # spring rest length [m]

# Layout — Y-up convention; ground at y=0, bodies hang below
GROUND_Y     = 0.0      # fixed anchor y position [m]
BODY1_Y      = GROUND_Y - REST_LENGTH          # body_1 equilibrium y [m]; precomputed once
BODY2_Y      = BODY1_Y  - REST_LENGTH          # body_2 equilibrium y [m]; precomputed once
BODY3_Y      = BODY2_Y  - REST_LENGTH          # body_3 equilibrium y [m]; precomputed once

# Render cadence: number of physics steps between rendered frames
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# === System & gravity ===
# ChSystemNSC — pure jointed MBS with no contact; collision system omitted per
# codegen_rules (pure jointed MBS, no contact shapes)
sys = chrono.ChSystemNSC()
sys.SetGravityY()   # (0, -9.81, 0)

# === Bodies ===
# Fixed ground body — acts as the top anchor for the spring chain
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0, GROUND_Y, 0))
ground_shape = chrono.ChVisualShapeBox(0.3, 0.1, 0.3)
ground_shape.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
ground.AddVisualShape(ground_shape)
sys.AddBody(ground)

# body_1 — first mass hanging from ground
body_1 = chrono.ChBody()
body_1.SetMass(BODY_MASS)
body_1.SetInertiaXX(chrono.ChVector3d(BODY_INERTIA, BODY_INERTIA, BODY_INERTIA))
body_1.SetPos(chrono.ChVector3d(0, BODY1_Y, 0))
sph1 = chrono.ChVisualShapeSphere(BODY_RADIUS)
sph1.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
body_1.AddVisualShape(sph1)
sys.AddBody(body_1)

# body_2 — second mass hanging from body_1
body_2 = chrono.ChBody()
body_2.SetMass(BODY_MASS)
body_2.SetInertiaXX(chrono.ChVector3d(BODY_INERTIA, BODY_INERTIA, BODY_INERTIA))
body_2.SetPos(chrono.ChVector3d(0, BODY2_Y, 0))
sph2 = chrono.ChVisualShapeSphere(BODY_RADIUS)
sph2.SetColor(chrono.ChColor(0.2, 0.8, 0.2))
body_2.AddVisualShape(sph2)
sys.AddBody(body_2)

# body_3 — third mass hanging from body_2
body_3 = chrono.ChBody()
body_3.SetMass(BODY_MASS)
body_3.SetInertiaXX(chrono.ChVector3d(BODY_INERTIA, BODY_INERTIA, BODY_INERTIA))
body_3.SetPos(chrono.ChVector3d(0, BODY3_Y, 0))
sph3 = chrono.ChVisualShapeSphere(BODY_RADIUS)
sph3.SetColor(chrono.ChColor(0.2, 0.2, 0.8))
body_3.AddVisualShape(sph3)
sys.AddBody(body_3)

# === Joints / Springs ===
# Spring connecting ground → body_1 (original spring)
spring_0_1 = chrono.ChLinkTSDA()
spring_0_1.Initialize(
    ground, body_1, True,
    chrono.ChVector3d(0, 0, 0),   # attachment on ground (body-local origin)
    chrono.ChVector3d(0, 0, 0),   # attachment on body_1 (body-local origin)
)
spring_0_1.SetRestLength(REST_LENGTH)
spring_0_1.SetSpringCoefficient(SPRING_K)
spring_0_1.SetDampingCoefficient(DAMPING_C)
spring_0_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))
sys.AddLink(spring_0_1)

# Spring connecting body_1 → body_2
spring_1_2 = chrono.ChLinkTSDA()
spring_1_2.Initialize(
    body_1, body_2, True,
    chrono.ChVector3d(0, 0, 0),   # attachment on body_1 (body-local origin)
    chrono.ChVector3d(0, 0, 0),   # attachment on body_2 (body-local origin)
)
spring_1_2.SetRestLength(REST_LENGTH)
spring_1_2.SetSpringCoefficient(SPRING_K)
spring_1_2.SetDampingCoefficient(DAMPING_C)
spring_1_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))
sys.AddLink(spring_1_2)

# Spring connecting body_2 → body_3
spring_2_3 = chrono.ChLinkTSDA()
spring_2_3.Initialize(
    body_2, body_3, True,
    chrono.ChVector3d(0, 0, 0),   # attachment on body_2 (body-local origin)
    chrono.ChVector3d(0, 0, 0),   # attachment on body_3 (body-local origin)
)
spring_2_3.SetRestLength(REST_LENGTH)
spring_2_3.SetSpringCoefficient(SPRING_K)
spring_2_3.SetDampingCoefficient(DAMPING_C)
spring_2_3.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))
sys.AddLink(spring_2_3)

# === Visualization ===
# Full Irrlicht block: Initialize FIRST, then add scene elements after
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Mass-Spring-Damper Chain (3 Bodies)")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(4, -2, 0), chrono.ChVector3d(0, -2, 0))
vis.AddTypicalLights()

# === Review-only setup ===

# === Main loop ===
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
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise
finally:
    pass
