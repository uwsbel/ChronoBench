"""
Mass-Spring-Damper Simulation with Custom Force Functor (PyChrono 9.0.x, Irrlicht)

Models two independent mass-spring-damper systems attached to a fixed ground body:
  - body_1: connected via spring_1 using direct spring/damping coefficients
  - body_2: connected via spring_2 using a custom MySpringForce functor
             (spring_coef=50, damping_coef=1)

System type: ChSystemNSC (pure jointed MBS, no collision).
Expected behavior: both bodies oscillate vertically; spring_2 uses the custom
functor to compute the same spring-damper force analytically.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants ===
SPRING_K     = 50.0    # spring stiffness [N/m]
DAMPING_C    = 1.0     # damping coefficient [N·s/m]
REST_LENGTH  = 1.0     # spring rest length [m]
BODY_MASS    = 1.0     # mass of each oscillating body [kg]
BODY_SIZE    = 0.1     # sphere radius for bodies [m]

TIME_STEP    = 1e-3    # physics step [s]
SIM_END      = 10.0    # simulation end time [s]
RENDER_FPS   = 50.0
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once


# === Custom Force Functor ===
class MySpringForce(chrono.ForceFunctor):
    """Custom spring-damper functor: F = -spring_coef*(length - rest) - damping_coef*vel"""

    def __init__(self, spring_coef, damping_coef):
        chrono.ForceFunctor.__init__(self)  # MUST call base ctor
        self.spring_coef  = spring_coef
        self.damping_coef = damping_coef

    def evaluate(self, time, rest_length, length, vel, link):
        return -self.spring_coef * (length - rest_length) - self.damping_coef * vel


# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
# Pure jointed MBS — no contact/collision; SetCollisionSystemType omitted per rules


# === Ground body ===
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetName("ground")
sys.AddBody(ground)

# Visual sphere sph_1 on ground at origin (0, 0, 0)
sph_1 = chrono.ChVisualShapeSphere(0.05)
sph_1.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
ground.AddVisualShape(sph_1, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))

# Visual sphere sph_2 on ground at (1, 0, 0) — mirrors sph_1
sph_2 = chrono.ChVisualShapeSphere(0.05)
sph_2.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
ground.AddVisualShape(sph_2, chrono.ChFramed(chrono.ChVector3d(1, 0, 0)))


# === Bodies ===
# body_1 — hangs from ground at x=0, initial position displaced from rest
body_1 = chrono.ChBody()
body_1.SetName("body_1")
body_1.SetMass(BODY_MASS)
body_1.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
body_1.SetPos(chrono.ChVector3d(0, -REST_LENGTH - 0.1, 0))  # slight displacement
sys.AddBody(body_1)

sph_body1 = chrono.ChVisualShapeSphere(BODY_SIZE)
sph_body1.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
body_1.AddVisualShape(sph_body1)

# body_2 — hangs from ground at x=1, mirrors body_1
body_2 = chrono.ChBody()
body_2.SetName("body_2")
body_2.SetMass(BODY_MASS)
body_2.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
body_2.SetPos(chrono.ChVector3d(1, -REST_LENGTH - 0.1, 0))  # slight displacement
sys.AddBody(body_2)

sph_body2 = chrono.ChVisualShapeSphere(BODY_SIZE)
sph_body2.SetColor(chrono.ChColor(0.2, 0.2, 0.8))
body_2.AddVisualShape(sph_body2)


# === Springs ===
# spring_1: body_1 <-> ground, direct spring and damping coefficients
spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground, True,
                    chrono.ChVector3d(0, 0, 0),     # attachment on body_1 (local)
                    chrono.ChVector3d(0, 0, 0))     # attachment on ground (local)
spring_1.SetRestLength(REST_LENGTH)
spring_1.SetSpringCoefficient(SPRING_K)
spring_1.SetDampingCoefficient(DAMPING_C)
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))
sys.AddLink(spring_1)

# spring_2: body_2 <-> ground, custom MySpringForce functor
my_force = MySpringForce(SPRING_K, DAMPING_C)
spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_2, ground, True,
                    chrono.ChVector3d(0, 0, 0),     # attachment on body_2 (local)
                    chrono.ChVector3d(1, 0, 0))     # attachment on ground at (1,0,0)
spring_2.SetRestLength(REST_LENGTH)
spring_2.RegisterForceFunctor(my_force)
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))
sys.AddLink(spring_2)


# === Visualization — full Irrlicht block (Initialize first, then scene elements) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Mass-Spring-Damper with Custom Force Functor")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()                                          # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3.0, 1.0, 3.0),
              chrono.ChVector3d(0.5, -1.0, 0.0))         # AFTER Initialize
vis.AddTypicalLights()


# === Review-only setup ===


# === Main loop ===
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
