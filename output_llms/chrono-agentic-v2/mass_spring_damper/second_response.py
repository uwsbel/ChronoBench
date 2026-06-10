"""
Mass-spring-damper simulation with two bodies and a custom force functor.

System: ChSystemNSC (Y-up, gravity disabled for vertical spring-mass demo)
Bodies: ground (fixed), body_1, body_2 — each connected to ground via a ChLinkTSDA spring.
- spring_1 uses direct spring/damping coefficients.
- spring_2 uses a custom MySpringForce ForceFunctor (spring_coef=50, damping_coef=1).
Ground has two visual sphere markers: sph_1 at (0,0,0) and sph_2 at (1,0,0).
Expected behavior: both masses oscillate on their respective springs; spring_2 force
is computed via the custom functor producing the same physics as a standard spring-damper.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Parameters ===
time_step = 1e-3          # physics timestep [s]
sim_end   = 10.0          # simulation end time [s]
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# Spring / damping coefficients for spring_1 (direct)
spring_coef_1  = 50.0
damping_coef_1 = 1.0

# Spring rest length (same for both springs)
rest_length = 1.5

# Body parameters
body_mass    = 1.0
body_inertia = 0.1

# Positions
ground_pos = chrono.ChVector3d(0, 0, 0)
body1_pos  = chrono.ChVector3d(0, rest_length, 0)   # hanging below attachment on Y axis
body2_pos  = chrono.ChVector3d(1, rest_length, 0)   # mirrored at x=1

# === Custom force functor for spring_2 ===
class MySpringForce(chrono.ForceFunctor):
    """Custom spring-damper force functor: F = -spring_coef*delta - damping_coef*vel."""
    def __init__(self, spring_coef: float, damping_coef: float) -> None:
        chrono.ForceFunctor.__init__(self)
        self._k = spring_coef    # cache: spring coefficient
        self._c = damping_coef   # cache: damping coefficient

    def evaluate(self, time, rest_len, length, vel, link):  # noqa: N802
        deformation = length - rest_len
        return -self._k * deformation - self._c * vel


# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  # gravity off; springs dominate

# === Bodies ===
# Ground body (fixed)
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(ground_pos)
sys.AddBody(ground)

# Visual sphere on ground at (0,0,0) — sph_1
sph_1 = chrono.ChVisualShapeSphere(0.1)
sph_1.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
ground.AddVisualShape(sph_1, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))

# Visual sphere on ground at (1,0,0) — sph_2 (mirrors sph_1)
sph_2 = chrono.ChVisualShapeSphere(0.1)
sph_2.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
ground.AddVisualShape(sph_2, chrono.ChFramed(chrono.ChVector3d(1, 0, 0)))

# body_1: mass oscillating on spring_1
body_1 = chrono.ChBody()
body_1.SetMass(body_mass)
body_1.SetInertiaXX(chrono.ChVector3d(body_inertia, body_inertia, body_inertia))
body_1.SetPos(body1_pos)
sys.AddBody(body_1)

# Visual box for body_1
box_1 = chrono.ChVisualShapeBox(0.4, 0.4, 0.4)
box_1.SetColor(chrono.ChColor(0.2, 0.6, 0.2))
body_1.AddVisualShape(box_1)

# body_2: mass oscillating on spring_2 (mirrors body_1 at x=1)
body_2 = chrono.ChBody()
body_2.SetMass(body_mass)
body_2.SetInertiaXX(chrono.ChVector3d(body_inertia, body_inertia, body_inertia))
body_2.SetPos(body2_pos)
sys.AddBody(body_2)

# Visual box for body_2
box_2 = chrono.ChVisualShapeBox(0.4, 0.4, 0.4)
box_2.SetColor(chrono.ChColor(0.2, 0.4, 0.8))
body_2.AddVisualShape(box_2)

# === Springs (ChLinkTSDA) ===
# spring_1: direct spring/damping coefficients between body_1 and ground
spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(
    body_1, ground,
    True,                                    # use local frames
    chrono.ChVector3d(0, 0, 0),              # attachment point on body_1 (local)
    chrono.ChVector3d(0, rest_length, 0),    # attachment point on ground (local = world here)
)
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef_1)
spring_1.SetDampingCoefficient(damping_coef_1)
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))
sys.AddLink(spring_1)

# spring_2: custom MySpringForce functor between body_2 and ground
spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(
    body_2, ground,
    True,
    chrono.ChVector3d(0, 0, 0),              # attachment on body_2 (local)
    chrono.ChVector3d(1, rest_length, 0),    # attachment on ground (local = world here)
)
spring_2.SetRestLength(rest_length)
my_force = MySpringForce(spring_coef=50.0, damping_coef=1.0)
spring_2.RegisterForceFunctor(my_force)
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))
sys.AddLink(spring_2)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Mass-Spring-Damper: Two Bodies with Custom Force Functor")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, 2, 4), chrono.ChVector3d(0.5, 1.5, 0))
vis.AddTypicalLights()

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
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
