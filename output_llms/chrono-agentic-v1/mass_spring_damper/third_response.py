"""
Mass-Spring-Damper Chain Simulation (3-body system).

Models a vertical chain of three masses (body_1, body_2, body_3) connected in
series by spring-damper links: ground -> spring1 -> body_1 -> spring2 -> body_2
-> spring3 -> body_3. All bodies are constrained to slide along a vertical guide
(prismatic joint to ground) so the system oscillates purely in the Y direction.

System: ChSystemNSC (no contact, pure jointed MBS with springs and prismatic guides).
Gravity: Y-up convention, (0, -9.81, 0).
Expected behavior: all three masses exhibit damped vertical oscillations.
"""

import math
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants ===
# Physics
time_step = 1e-3          # integration step size [s]
sim_end   = 10.0          # simulation duration [s]
render_fps = 50.0         # review video frame rate [fps]
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# Body masses [kg]
mass_1 = 1.0
mass_2 = 1.5
mass_3 = 0.8

# Body sizes (box shapes) [m]
body_size = 0.2   # uniform cube edge for all three mass blocks

# Spring-damper parameters
spring_k  = 100.0  # spring stiffness [N/m]
damping_c = 5.0    # damping coefficient [N·s/m]
rest_len  = 1.0    # natural rest length for each spring [m]

# Vertical positions (Y-up, chain hangs downward from ground origin at Y=0)
# body_1 sits at rest_len below the ground anchor
y_body1 = -(rest_len + body_size / 2.0)
y_body2 = y_body1 - rest_len - body_size
y_body3 = y_body2 - rest_len - body_size

# Initial displacement (to excite oscillation)
y_body1 += 0.1
y_body2 += 0.05

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
# Pure jointed MBS with no collision shapes — SetCollisionSystemType is omitted intentionally

# === Bodies ===

# Ground anchor body (fixed reference)
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetName("ground")
ground.SetPos(chrono.ChVector3d(0, 0, 0))
ground_vis = chrono.ChVisualShapeBox(0.4, 0.1, 0.4)
ground_vis.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
ground.AddVisualShape(ground_vis)
sys.AddBody(ground)

# body_1: first mass in the chain
body_1 = chrono.ChBody()
body_1.SetName("body_1")
body_1.SetMass(mass_1)
I1 = (1.0 / 6.0) * mass_1 * body_size**2  # solid cube inertia
body_1.SetInertiaXX(chrono.ChVector3d(I1, I1, I1))
body_1.SetPos(chrono.ChVector3d(0, y_body1, 0))
body_1.EnableCollision(False)
vis1 = chrono.ChVisualShapeBox(body_size, body_size, body_size)
vis1.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
body_1.AddVisualShape(vis1)
sys.AddBody(body_1)

# body_2: second mass in the chain
body_2 = chrono.ChBody()
body_2.SetName("body_2")
body_2.SetMass(mass_2)
I2 = (1.0 / 6.0) * mass_2 * body_size**2
body_2.SetInertiaXX(chrono.ChVector3d(I2, I2, I2))
body_2.SetPos(chrono.ChVector3d(0, y_body2, 0))
body_2.EnableCollision(False)
vis2 = chrono.ChVisualShapeBox(body_size, body_size, body_size)
vis2.SetColor(chrono.ChColor(0.2, 0.6, 0.2))
body_2.AddVisualShape(vis2)
sys.AddBody(body_2)

# body_3: third mass in the chain
body_3 = chrono.ChBody()
body_3.SetName("body_3")
body_3.SetMass(mass_3)
I3 = (1.0 / 6.0) * mass_3 * body_size**2
body_3.SetInertiaXX(chrono.ChVector3d(I3, I3, I3))
body_3.SetPos(chrono.ChVector3d(0, y_body3, 0))
body_3.EnableCollision(False)
vis3 = chrono.ChVisualShapeBox(body_size, body_size, body_size)
vis3.SetColor(chrono.ChColor(0.2, 0.2, 0.8))
body_3.AddVisualShape(vis3)
sys.AddBody(body_3)

# === Joints / constraints ===
# Prismatic joints: each body slides vertically (along Y axis = guide)
# ChLinkLockPrismatic local +Z is the sliding axis; rotate so +Z -> +Y
q_slide = chrono.QuatFromAngleAxis(chrono.CH_PI_2, chrono.VECT_X)  # maps Z -> Y

pris1 = chrono.ChLinkLockPrismatic()
pris1.Initialize(body_1, ground,
                 chrono.ChFramed(body_1.GetPos(), q_slide))
sys.AddLink(pris1)

pris2 = chrono.ChLinkLockPrismatic()
pris2.Initialize(body_2, ground,
                 chrono.ChFramed(body_2.GetPos(), q_slide))
sys.AddLink(pris2)

pris3 = chrono.ChLinkLockPrismatic()
pris3.Initialize(body_3, ground,
                 chrono.ChFramed(body_3.GetPos(), q_slide))
sys.AddLink(pris3)

# === Springs ===
# Spring between ground anchor and body_1 (body_1 and body_2 from the original config)
# Spring between body_1 and body_2 (new spring per prompt)
# Spring between body_2 and body_3 (new spring per prompt)

# ground -> body_1 (original spring, reconstructed)
spring0 = chrono.ChLinkTSDA()
spring0.Initialize(
    ground, body_1, True,
    chrono.ChVector3d(0, 0, 0),        # attachment on ground (local coords)
    chrono.ChVector3d(0, body_size / 2.0, 0)  # top of body_1 (local coords)
)
spring0.SetRestLength(rest_len)
spring0.SetSpringCoefficient(spring_k)
spring0.SetDampingCoefficient(damping_c)
spring0.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 15, 10))
sys.AddLink(spring0)

# body_1 -> body_2 (new spring per prompt)
spring1 = chrono.ChLinkTSDA()
spring1.Initialize(
    body_1, body_2, True,
    chrono.ChVector3d(0, -body_size / 2.0, 0),  # bottom of body_1 (local)
    chrono.ChVector3d(0, body_size / 2.0, 0)    # top of body_2 (local)
)
spring1.SetRestLength(rest_len)
spring1.SetSpringCoefficient(spring_k)
spring1.SetDampingCoefficient(damping_c)
spring1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 15, 10))
sys.AddLink(spring1)

# body_2 -> body_3 (new spring per prompt)
spring2 = chrono.ChLinkTSDA()
spring2.Initialize(
    body_2, body_3, True,
    chrono.ChVector3d(0, -body_size / 2.0, 0),  # bottom of body_2 (local)
    chrono.ChVector3d(0, body_size / 2.0, 0)    # top of body_3 (local)
)
spring2.SetRestLength(rest_len)
spring2.SetSpringCoefficient(spring_k)
spring2.SetDampingCoefficient(damping_c)
spring2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 15, 10))
sys.AddLink(spring2)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Mass-Spring-Damper Chain — 3-Body System")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, -3, 1), chrono.ChVector3d(0, -3, 0))
vis.AddTypicalLights()
vis.AddGrid(
    0.5, 0.5, 20, 20,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4)
)

# === Review-only setup ===

# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t = sys.GetChTime()
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
