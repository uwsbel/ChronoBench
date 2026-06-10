"""
Mass-Spring-Damper Simulation — PyChrono 9.0.x / Irrlicht

Models a single mass-spring-damper system using ChLinkTSDA to connect
a dynamic mass body to the fixed ground. The mass is displaced from the
spring rest length and released; it oscillates while the damper dissipates
energy.

System type : ChSystemNSC (pure jointed MBS — no contact/collision)
Bodies      : ground (fixed), mass block
Link        : ChLinkTSDA (linear spring + damper)
Visualization: Irrlicht window with spring visual shape and box mass shape
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants ===
# Physics parameters
MASS = 1.0           # mass [kg]
SPRING_K = 50.0      # spring stiffness [N/m]
DAMPING_C = 2.0      # damping coefficient [N·s/m]
REST_LENGTH = 1.0    # spring natural/rest length [m]
INIT_DISPLACEMENT = 0.3  # initial displacement beyond rest length [m]

# Geometry
GROUND_POS_Y = 0.0
MASS_INIT_Y = GROUND_POS_Y - (REST_LENGTH + INIT_DISPLACEMENT)  # Y-down spring
MASS_SIZE = 0.2      # box half-size for display [m]

# Simulation timing
TIME_STEP = 1e-3      # [s]
SIM_END = 10.0        # [s]
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# === System & gravity ===
# ChSystemNSC for rigid-body mass-spring (no contact — omit SetCollisionSystemType)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # Y-down gravity

# === Bodies ===
# Ground body — fixed anchor for the spring
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0.0, GROUND_POS_Y, 0.0))
# Add a small visual box at the anchor to make the attachment visible
anchor_shape = chrono.ChVisualShapeBox(0.15, 0.05, 0.15)
anchor_shape.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
ground.AddVisualShape(anchor_shape, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
sys.AddBody(ground)

# Mass body — hangs below the ground anchor, oscillates vertically
mass_body = chrono.ChBody()
mass_body.SetMass(MASS)
mass_body.SetInertiaXX(chrono.ChVector3d(
    (1.0 / 6.0) * MASS * (2 * MASS_SIZE) ** 2,  # solid box Ixx = 1/6 * m * (2a)^2 for square
    (1.0 / 6.0) * MASS * (2 * MASS_SIZE) ** 2,
    (1.0 / 6.0) * MASS * (2 * MASS_SIZE) ** 2,
))
mass_body.SetPos(chrono.ChVector3d(0.0, MASS_INIT_Y, 0.0))
mass_body.EnableCollision(False)
mass_shape = chrono.ChVisualShapeBox(2 * MASS_SIZE, 2 * MASS_SIZE, 2 * MASS_SIZE)
mass_shape.SetColor(chrono.ChColor(0.2, 0.5, 0.9))
mass_body.AddVisualShape(mass_shape)
sys.AddBody(mass_body)

# === Spring / damper link (ChLinkTSDA) ===
# Attach spring between ground (anchor at origin) and top of mass block
# Use body-local True so attachment points are in each body's local frame
spring = chrono.ChLinkTSDA()
spring.Initialize(
    ground,   # body 1 (fixed anchor)
    mass_body,  # body 2 (moving mass)
    True,  # positions in body-local frames
    chrono.ChVector3d(0.0, 0.0, 0.0),   # attachment point on ground (local origin)
    chrono.ChVector3d(0.0, MASS_SIZE, 0.0),  # attachment on top face of mass (local)
)
spring.SetRestLength(REST_LENGTH)
spring.SetSpringCoefficient(SPRING_K)
spring.SetDampingCoefficient(DAMPING_C)

# Visual shape for the spring coil — added to the link
spring_visual = chrono.ChVisualShapeSpring(0.05, 80, 15)
spring_visual.SetColor(chrono.ChColor(0.9, 0.3, 0.1))
spring.AddVisualShape(spring_visual)
sys.AddLink(spring)

# === Visualization ===
# Full Irrlicht block: Initialize() first, then all scene elements
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Mass-Spring-Damper — PyChrono")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)  # Y-up scene
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(2.5, -1.0, 2.5),   # eye position
    chrono.ChVector3d(0.0, -1.0, 0.0),   # look-at target (mid-spring region)
)
vis.AddTypicalLights()
vis.AddGrid(
    0.5, 0.5, 20, 20,
    chrono.ChCoordsysd(chrono.ChVector3d(0, GROUND_POS_Y, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

# === Review-only setup ===

# === Main loop ===
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
except (RuntimeError, ValueError) as exc:   # solver divergence / bad parameter
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing (review-only) ===
