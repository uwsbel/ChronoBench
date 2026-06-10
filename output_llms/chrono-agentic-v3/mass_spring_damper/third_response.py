"""
Mass-Spring-Damper System with Three Bodies.

Models three masses (body_1, body_2, body_3) connected in series by spring-damper
links: ground anchors body_1 via a spring-damper, body_1 connects to body_2 via
another spring-damper, and body_2 connects to body_3 via a third spring-damper.
All bodies oscillate vertically under gravity. System type: NSC. Expected behavior:
coupled oscillations of all three masses with damped vertical motion.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants ===
time_step = 1e-3          # integration step [s]
sim_end   = 10.0          # simulation duration [s]
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# Body geometry
BODY_MASS   = 1.0       # [kg]
BODY_SIZE   = 0.2       # box half-extent [m]

# Spring-damper parameters (same as original demo)
SPRING_K    = 50.0      # spring stiffness [N/m]
DAMPING_C   = 1.0       # damping coefficient [Ns/m]
REST_LEN    = 1.0       # rest length [m]

# Body positions (vertical stack, Y-up)
POS_BODY1 = chrono.ChVector3d(0.0, -1.0, 0.0)
POS_BODY2 = chrono.ChVector3d(0.0, -2.5, 0.0)
POS_BODY3 = chrono.ChVector3d(0.0, -4.0, 0.0)

# Spring attachment points on ground anchor
ANCHOR_POS = chrono.ChVector3d(0.0,  0.0, 0.0)

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
# Pure jointed/spring MBS — no collision shapes, so SetCollisionSystemType is omitted

# === Bodies ===
# Ground anchor (fixed reference)
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0, 0, 0))
# Visual: small fixed sphere to indicate anchor
anchor_vis = chrono.ChVisualShapeSphere(0.05)
anchor_vis.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
ground.AddVisualShape(anchor_vis, chrono.ChFramed(ANCHOR_POS, chrono.QUNIT))
sys.AddBody(ground)

# body_1
body_1 = chrono.ChBody()
body_1.SetMass(BODY_MASS)
body_1.SetInertiaXX(chrono.ChVector3d(0.2, 0.2, 0.2))
body_1.SetPos(POS_BODY1)
box1 = chrono.ChVisualShapeBox(BODY_SIZE * 2, BODY_SIZE * 2, BODY_SIZE * 2)
box1.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
body_1.AddVisualShape(box1)
sys.AddBody(body_1)

# body_2
body_2 = chrono.ChBody()
body_2.SetMass(BODY_MASS)
body_2.SetInertiaXX(chrono.ChVector3d(0.2, 0.2, 0.2))
body_2.SetPos(POS_BODY2)
box2 = chrono.ChVisualShapeBox(BODY_SIZE * 2, BODY_SIZE * 2, BODY_SIZE * 2)
box2.SetColor(chrono.ChColor(0.2, 0.8, 0.2))
body_2.AddVisualShape(box2)
sys.AddBody(body_2)

# body_3
body_3 = chrono.ChBody()
body_3.SetMass(BODY_MASS)
body_3.SetInertiaXX(chrono.ChVector3d(0.2, 0.2, 0.2))
body_3.SetPos(POS_BODY3)
box3 = chrono.ChVisualShapeBox(BODY_SIZE * 2, BODY_SIZE * 2, BODY_SIZE * 2)
box3.SetColor(chrono.ChColor(0.2, 0.2, 0.8))
body_3.AddVisualShape(box3)
sys.AddBody(body_3)

# === Springs / constraints ===
# Spring 1: ground anchor → body_1 (vertical, body_1 attachment at its top)
spring1 = chrono.ChLinkTSDA()
spring1.Initialize(
    ground, body_1, True,
    ANCHOR_POS,                              # attachment on ground (local)
    chrono.ChVector3d(0.0, BODY_SIZE, 0.0)   # attachment on body_1 top face (local)
)
spring1.SetRestLength(REST_LEN)
spring1.SetSpringCoefficient(SPRING_K)
spring1.SetDampingCoefficient(DAMPING_C)
spring1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))
sys.AddLink(spring1)

# Spring 2: body_1 → body_2
spring2 = chrono.ChLinkTSDA()
spring2.Initialize(
    body_1, body_2, True,
    chrono.ChVector3d(0.0, -BODY_SIZE, 0.0),  # body_1 bottom face (local)
    chrono.ChVector3d(0.0,  BODY_SIZE, 0.0)   # body_2 top face (local)
)
spring2.SetRestLength(REST_LEN)
spring2.SetSpringCoefficient(SPRING_K)
spring2.SetDampingCoefficient(DAMPING_C)
spring2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))
sys.AddLink(spring2)

# Spring 3: body_2 → body_3
spring3 = chrono.ChLinkTSDA()
spring3.Initialize(
    body_2, body_3, True,
    chrono.ChVector3d(0.0, -BODY_SIZE, 0.0),  # body_2 bottom face (local)
    chrono.ChVector3d(0.0,  BODY_SIZE, 0.0)   # body_3 top face (local)
)
spring3.SetRestLength(REST_LEN)
spring3.SetSpringCoefficient(SPRING_K)
spring3.SetDampingCoefficient(DAMPING_C)
spring3.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))
sys.AddLink(spring3)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Mass-Spring-Damper: Three Bodies")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, -2, 3), chrono.ChVector3d(0, -2, 0))
vis.AddTypicalLights()

# === Review-only setup ===

# === Main loop ===
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / bad spring state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
