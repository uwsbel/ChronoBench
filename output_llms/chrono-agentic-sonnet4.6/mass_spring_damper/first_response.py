"""
Mass-spring-damper simulation using PyChrono.

Models a single mass connected to a fixed ground via a ChLinkTSDA spring-damper.
System type: NSC (ChSystemNSC). The mass is initialized at an offset from the
spring rest length and oscillates under gravity with exponential decay from
the damper. Y-up convention; spring axis is vertical (Y-axis).

Expected behavior: the mass oscillates vertically about the spring equilibrium
position and gradually damps to rest.
"""

import os
import csv
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants ===
TIME_STEP = 1e-3          # physics timestep [s]
SIM_END   = 10.0          # simulation duration [s]
RENDER_FPS = 50.0         # Irrlicht render rate [fps]
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Mass body parameters
MASS          = 1.0       # [kg]
BOX_SIZE      = 0.2       # box full side length [m]
BOX_DENSITY   = MASS / (BOX_SIZE ** 3)  # density to yield target mass

# Spring-damper parameters (default linear spring-damper for ChLinkTSDA)
SPRING_K      = 50.0      # spring stiffness [N/m]
DAMPING_C     = 3.0       # damping coefficient [N·s/m]
REST_LENGTH   = 1.5       # natural (rest) length of the spring [m]
INITIAL_Y     = -0.8      # initial Y offset of mass below ground anchor [m]
                          # (places mass below rest-length position for oscillation)

# Ground anchor position (where spring attaches to fixed body)
ANCHOR_Y      = 0.5       # anchor Y coordinate [m]

# Mass spawn position: rest_length below the anchor
MASS_INIT_Y   = ANCHOR_Y - (REST_LENGTH + 0.3)  # 0.3 m below rest pos → initial stretch

# Camera placement
EYE    = chrono.ChVector3d(4, -2, 2)
TARGET = chrono.ChVector3d(0, -1, 0)

# === System & gravity ===  NSC, Y-up gravity
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Bodies ===

# Fixed ground body (anchor point for the spring)
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0, ANCHOR_Y, 0))
ground_vis = chrono.ChVisualShapeBox(0.3, 0.05, 0.3)
ground_vis.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
ground.AddVisualShape(ground_vis)
sys.AddBody(ground)

# Moving mass body
mass_body = chrono.ChBody()
mass_body.SetMass(MASS)
inertia_val = (1.0 / 6.0) * MASS * BOX_SIZE * BOX_SIZE  # solid box I = (1/6)mL² per axis
mass_body.SetInertiaXX(chrono.ChVector3d(inertia_val, inertia_val, inertia_val))
mass_body.SetPos(chrono.ChVector3d(0, MASS_INIT_Y, 0))
mass_vis = chrono.ChVisualShapeBox(BOX_SIZE, BOX_SIZE, BOX_SIZE)
mass_vis.SetColor(chrono.ChColor(0.6, 0.2, 0.2))
mass_body.AddVisualShape(mass_vis)
sys.AddBody(mass_body)

# === Joints / constraints ===
# ChLinkTSDA: spring-damper connecting ground anchor to mass body
# Initialize with body-local connection points
spring = chrono.ChLinkTSDA()
spring.Initialize(
    ground,     # body 1 (ground)
    mass_body,  # body 2 (mass)
    True,       # use body-local frames
    chrono.ChVector3d(0, 0, 0),            # attachment on ground (local origin)
    chrono.ChVector3d(0, 0, 0),            # attachment on mass body (local origin)
)
spring.SetRestLength(REST_LENGTH)
spring.SetSpringCoefficient(SPRING_K)
spring.SetDampingCoefficient(DAMPING_C)
# Add spring visual shape to the link (coil_radius, resolution, turns)
spring.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))
sys.AddLink(spring)

# === Visualization ===  Full Irrlicht scene
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Mass-Spring-Damper — PyChrono")
vis.Initialize()                          # Initialize FIRST (Irrlicht order)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(EYE, TARGET)               # AFTER Initialize
vis.AddTypicalLights()

# === Main loop ===


# CSV writer (review-only) — opened before the loop

frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t  = sys.GetChTime()         # cache: fetched once per step
            py = mass_body.GetPos().y    # cache: mass Y position
            vy = mass_body.GetPosDt().y  # cache: mass Y velocity
            sl = spring.GetLength()      # cache: current spring length
            sf = spring.GetForce()       # cache: current spring force
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad initial state
    import traceback
    traceback.print_exc()
    raise
