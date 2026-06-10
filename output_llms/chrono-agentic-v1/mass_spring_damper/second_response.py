"""
Mass-Spring-Damper Simulation with Custom Force Functor (PyChrono 9.0.x, Irrlicht).

Models two bodies (body_1 and body_2) each vertically suspended from the fixed
ground via a ChLinkTSDA spring-damper:
  - spring_1 connects body_1 to ground using direct spring/damping coefficients
    (spring_coef = 100, damping_coef = 5).
  - spring_2 connects body_2 to ground using a custom MySpringForce ForceFunctor
    (spring_coef = 50, damping_coef = 1).
Both bodies start displaced from equilibrium and oscillate freely under gravity
(Y-up convention). A visual sphere (sph_1, sph_2) is placed on the ground at
each body's X-position as a reference anchor marker.

System: ChSystemNSC (pure jointed MBS, no collision needed).
Expected behavior: both bodies oscillate vertically with exponential decay;
body_2 oscillates with lower stiffness and less damping (slower decay, lower freq).
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants ===
# Simulation timing
time_step = 1e-3                         # physics timestep [s]
sim_end   = 10.0                         # simulation end time [s]
render_fps = 30.0                        # review video frame rate [fps]
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# Spring / damper coefficients
SPRING_1_K  = 100.0   # spring_1 stiffness [N/m]
SPRING_1_C  = 5.0     # spring_1 damping [N·s/m]
SPRING_2_K  = 50.0    # spring_2 stiffness via custom functor [N/m]
SPRING_2_C  = 1.0     # spring_2 damping via custom functor [N·s/m]
REST_LENGTH = 1.5     # natural (rest) length of both springs [m]

# Body geometry / mass
BODY_MASS    = 1.0    # mass of each oscillating body [kg]
BODY_RADIUS  = 0.1    # visual radius for body spheres [m]
GROUND_SPH_R = 0.05   # visual radius for ground anchor spheres [m]

# Body 1: at x=0, displaced above rest position
BODY1_X = 0.0
BODY1_Y = -REST_LENGTH - 0.5  # initial displacement below ground attachment
BODY1_Z = 0.0

# Body 2: at x=1, mirroring body_1 configuration
BODY2_X = 1.0
BODY2_Y = -REST_LENGTH - 0.5  # same initial displacement
BODY2_Z = 0.0

# Ground attachment points (at Y=0)
ATTACH1_GROUND = chrono.ChVector3d(BODY1_X, 0.0, 0.0)
ATTACH2_GROUND = chrono.ChVector3d(BODY2_X, 0.0, 0.0)

# === Custom Force Functor ===
class MySpringForce(chrono.ForceFunctor):
    """Custom spring-damper force functor for spring_2 (body_2 to ground).

    Evaluates F = -k*(length - rest_length) - c*vel using custom parameters.
    """

    def __init__(self, spring_coef: float = SPRING_2_K, damping_coef: float = SPRING_2_C):
        chrono.ForceFunctor.__init__(self)  # MUST call base constructor
        self._k = spring_coef    # stiffness coefficient [N/m]
        self._c = damping_coef   # damping coefficient [N·s/m]

    def evaluate(self, time: float, rest_length: float, length: float,
                 vel: float, link: chrono.ChLinkTSDA) -> float:
        """Return spring-damper force: F = -k*(deformation) - c*velocity."""
        deformation = length - rest_length
        force = -self._k * deformation - self._c * vel
        return force


# === System & gravity ===
# ChSystemNSC: pure MBS with spring links, no collision contact needed
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # Y-up

# === Bodies ===
# --- Ground (fixed reference body) ---
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetMass(1.0)
ground.SetInertiaXX(chrono.ChVector3d(1.0, 1.0, 1.0))
ground.SetPos(chrono.ChVector3d(0, 0, 0))

# Visual sphere sph_1 on ground at (0, 0, 0) — anchor marker for body_1
sph_1 = chrono.ChVisualShapeSphere(GROUND_SPH_R)
sph_1.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
ground.AddVisualShape(sph_1, chrono.ChFramed(
    chrono.ChVector3d(BODY1_X, 0.0, 0.0), chrono.QUNIT))

# Visual sphere sph_2 on ground at (1, 0, 0) — anchor marker for body_2
sph_2 = chrono.ChVisualShapeSphere(GROUND_SPH_R)
sph_2.SetColor(chrono.ChColor(0.2, 0.2, 0.8))
ground.AddVisualShape(sph_2, chrono.ChFramed(
    chrono.ChVector3d(BODY2_X, 0.0, 0.0), chrono.QUNIT))

sys.AddBody(ground)

# --- Body 1: oscillating mass for spring_1 ---
body_1 = chrono.ChBody()
body_1.SetMass(BODY_MASS)
# Inertia for solid sphere: I = (2/5)*m*r^2
I1 = (2.0 / 5.0) * BODY_MASS * BODY_RADIUS ** 2
body_1.SetInertiaXX(chrono.ChVector3d(I1, I1, I1))
body_1.SetPos(chrono.ChVector3d(BODY1_X, BODY1_Y, BODY1_Z))
body_1.EnableCollision(False)

# Sphere visual shape for body_1
vs_body1 = chrono.ChVisualShapeSphere(BODY_RADIUS)
vs_body1.SetColor(chrono.ChColor(0.9, 0.3, 0.1))
body_1.AddVisualShape(vs_body1)
sys.AddBody(body_1)

# --- Body 2: oscillating mass for spring_2 (mirrors body_1 configuration) ---
body_2 = chrono.ChBody()
body_2.SetMass(BODY_MASS)
I2 = (2.0 / 5.0) * BODY_MASS * BODY_RADIUS ** 2  # same geometry as body_1
body_2.SetInertiaXX(chrono.ChVector3d(I2, I2, I2))
body_2.SetPos(chrono.ChVector3d(BODY2_X, BODY2_Y, BODY2_Z))
body_2.EnableCollision(False)

# Sphere visual shape for body_2 (mirrors body_1)
vs_body2 = chrono.ChVisualShapeSphere(BODY_RADIUS)
vs_body2.SetColor(chrono.ChColor(0.1, 0.3, 0.9))
body_2.AddVisualShape(vs_body2)
sys.AddBody(body_2)

# === Springs / Constraints ===
# --- spring_1: direct coefficient setup between body_1 and ground ---
spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(
    body_1, ground,
    True,                        # use body-local attachment points
    chrono.ChVector3d(0, 0, 0),  # attachment on body_1 (at its center)
    ATTACH1_GROUND               # attachment on ground
)
spring_1.SetRestLength(REST_LENGTH)
spring_1.SetSpringCoefficient(SPRING_1_K)
spring_1.SetDampingCoefficient(SPRING_1_C)
# Visual coil shape for spring_1
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.06, 80, 15))
sys.AddLink(spring_1)

# --- spring_2: custom force functor between body_2 and ground ---
spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(
    body_2, ground,
    True,                        # use body-local attachment points
    chrono.ChVector3d(0, 0, 0),  # attachment on body_2 (at its center)
    ATTACH2_GROUND               # attachment on ground
)
spring_2.SetRestLength(REST_LENGTH)
# Register the custom MySpringForce functor — replaces direct coefficient approach
my_spring_force = MySpringForce(SPRING_2_K, SPRING_2_C)
spring_2.RegisterForceFunctor(my_spring_force)
# Visual coil shape for spring_2
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.06, 80, 15))
sys.AddLink(spring_2)

# === Visualization ===
# Full Irrlicht block: window + Initialize + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Mass-Spring-Damper: Two Bodies with Custom Force Functor")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)  # Y-up scene
vis.Initialize()                                    # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(3.0, -1.5, 3.0),   # eye: side view of both bodies
    chrono.ChVector3d(0.5, -1.5, 0.0)    # target: midpoint between body_1 and body_2
)
vis.AddTypicalLights()
vis.AddGrid(
    0.5, 0.5,
    20, 20,
    chrono.ChCoordsysd(chrono.ChVector3d(0.5, 0, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4)
)

# === Review-only setup ===

# === Main loop ===
frame = 0  # precomputed once: consecutive frame counter for review video


try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
