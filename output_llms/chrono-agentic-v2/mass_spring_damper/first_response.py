"""
Mass-Spring-Damper Simulation
==============================
Models a single mass-spring-damper system using ChLinkTSDA to connect
a moving mass body to a fixed ground body. The system uses a ChSystemNSC
(Non-Smooth Contact, no contact in this scene). Under gravity the mass
oscillates vertically about the spring's natural equilibrium. Expected
behaviour: damped sinusoidal oscillation converging toward the static
equilibrium position.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants ===
TIME_STEP = 1e-3            # integration step [s]
SIM_END   = 10.0            # simulation duration [s]
RENDER_FPS = 50.0           # frames per second for review capture
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Mass geometry / physics
MASS_SIZE  = 0.4            # box side length [m]
MASS_MASS  = 1.0            # [kg]
MASS_INERT = (1.0/6.0) * MASS_MASS * MASS_SIZE**2  # solid-cube MOI [kg·m²]

# Spring attachment heights (Y-up world)
GROUND_Y    = 0.0           # ground level
SPRING_TOP_Y = 3.0          # anchor on ground body (above ground)
MASS_INIT_Y  = 1.5          # initial mass centre position (spring compressed)
SPRING_REST  = 1.2          # natural (rest) length [m]

# Default spring-damper parameters
SPRING_K = 50.0             # spring stiffness [N/m]
DAMPING_C = 1.0             # damping coefficient [N·s/m]

# Camera placement
CAM_EYE    = chrono.ChVector3d(0, -6, 3)
CAM_TARGET = chrono.ChVector3d(0,  0, 1.5)

# === System & gravity (Y-up, NSC, no contact) ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Bodies ===
# Ground — fixed, provides spring attachment point above floor
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetMass(1.0)
ground.SetInertiaXX(chrono.ChVector3d(1.0, 1.0, 1.0))
ground.SetPos(chrono.ChVector3d(0, GROUND_Y, 0))
ground.EnableCollision(False)
# Visual: thin slab at floor level
floor_shape = chrono.ChVisualShapeBox(2.0, 0.05, 2.0)
floor_shape.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
ground.AddVisualShape(floor_shape, chrono.ChFramed(chrono.ChVector3d(0, -0.025, 0), chrono.QUNIT))
sys.AddBody(ground)

# Moving mass body
mass_body = chrono.ChBody()
mass_body.SetMass(MASS_MASS)
mass_body.SetInertiaXX(chrono.ChVector3d(MASS_INERT, MASS_INERT, MASS_INERT))
mass_body.SetPos(chrono.ChVector3d(0, MASS_INIT_Y, 0))
mass_body.EnableCollision(False)
# Visual: box representing the mass
mass_shape = chrono.ChVisualShapeBox(MASS_SIZE, MASS_SIZE, MASS_SIZE)
mass_shape.SetColor(chrono.ChColor(0.2, 0.5, 0.8))
mass_body.AddVisualShape(mass_shape)
sys.AddBody(mass_body)

# === Joints / constraints — spring-damper link ===
spring = chrono.ChLinkTSDA()
# Initialize with body-local attachment points (True flag)
# Attachment on ground: world point (0, SPRING_TOP_Y, 0)  → local (0, SPRING_TOP_Y, 0) since ground origin is at 0
# Attachment on mass:   world point (0, MASS_INIT_Y + MASS_SIZE/2, 0) → top face of mass box
spring.Initialize(
    mass_body, ground, True,
    chrono.ChVector3d(0.0,  MASS_SIZE / 2.0, 0.0),   # top of mass (body-local)
    chrono.ChVector3d(0.0,  SPRING_TOP_Y,    0.0),   # anchor on ground (body-local = world)
)
spring.SetRestLength(SPRING_REST)
spring.SetSpringCoefficient(SPRING_K)
spring.SetDampingCoefficient(DAMPING_C)
# Visual spring coil on the link
spring.AddVisualShape(chrono.ChVisualShapeSpring(0.08, 80, 15))
sys.AddLink(spring)

# === Visualization — full Irrlicht scene ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Mass-Spring-Damper (ChLinkTSDA)")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(CAM_EYE, CAM_TARGET)
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, GROUND_Y, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop ===

frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t  = sys.GetChTime()
            py = mass_body.GetPos().y  # cache: mass Y position this step
            vy = mass_body.GetPosDt().y  # cache: mass Y velocity this step
            sl = spring.GetLength()       # cache: current spring length
            sf = spring.GetForce()        # cache: spring-damper scalar force
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
