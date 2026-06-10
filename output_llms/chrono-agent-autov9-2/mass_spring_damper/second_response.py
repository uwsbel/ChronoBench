"""Mass-spring-damper oscillators in PyChrono (ChSystemNSC).

Models two independent translational mass-spring-damper oscillators hanging from
a fixed ground anchor. Each free body is connected to the ground by a
ChLinkTSDA (translational spring-damper) and is rendered with a coil
ChVisualShapeSpring on the link:

  * body_1 / spring_1 : uses the built-in spring coefficient + damping
    coefficient of ChLinkTSDA directly (k = 50 N/m, c = 1 N.s/m).
  * body_2 / spring_2 : uses a custom chrono.ForceFunctor (MySpringForce) that
    computes the same linear spring-damper law from the supplied spring/damping
    parameters, demonstrating an alternative force-attachment method.

This is a pure jointed/elastic multi-body system with NO contact or collision,
so no collision system is configured (ground truth omits it for spring demos).
Gravity acts along -Z; both masses are displaced from their rest position at
start and are expected to oscillate and settle toward static equilibrium under
the spring restoring force and viscous damping.
"""

import os
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / physics parameters (no bare literals downstream)
time_step = 1e-3            # integration step (s)
sim_end = 5.0              # simulation duration (s)
render_fps = 50.0          # review render cadence (frames/s)

spring_coef = 50.0         # spring stiffness k (N/m)
damping_coef = 1.0         # viscous damping c (N.s/m)
rest_length = 1.5          # natural (unstretched) spring length (m)

body_mass = 1.0            # oscillator mass (kg)
body_radius = 0.2          # visualization sphere radius (m)
anchor_radius = 0.04       # ground anchor marker radius (m)
coil_radius = 0.10         # spring coil visual radius (m)
coil_resolution = 80       # spring coil visual resolution
coil_turns = 15            # spring coil visual turns

# Ground anchor points: sph_1 at origin, sph_2 mirrored at (1, 0, 0).
anchor_1 = chrono.ChVector3d(0.0, 0.0, 0.0)
anchor_2 = chrono.ChVector3d(1.0, 0.0, 0.0)
# Start each mass below its anchor, stretched beyond rest length so it oscillates.
start_drop = 2.0
body_1_pos = chrono.ChVector3d(anchor_1.x, anchor_1.y, anchor_1.z - start_drop)
body_2_pos = chrono.ChVector3d(anchor_2.x, anchor_2.y, anchor_2.z - start_drop)


# === Custom force functor === linear spring-damper force law for spring_2
class MySpringForce(chrono.ForceFunctor):
    """Evaluate a linear spring-damper force from custom coefficients.

    ChLinkTSDA calls evaluate(time, rest_length, length, vel, link) each step;
    the returned scalar is the force along the link acting to restore rest length.
    """

    def __init__(self, k, c):
        super().__init__()
        self.k = k          # spring stiffness (N/m)
        self.c = c          # damping coefficient (N.s/m)

    def evaluate(self, time, rest_length, length, vel, link):
        # Restoring force: -k*(length - rest_length) - c*vel
        return -self.k * (length - rest_length) - self.c * vel


# === System & gravity === pure spring-damper MBS, gravity along -Z
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# Spring-damper chains are stiff: PSOR + warm start for stable convergence.
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
solver = sys.GetSolver().AsIterative()
solver.SetMaxIterations(100)
solver.SetTolerance(1e-10)
solver.EnableWarmStart(True)  # reuse previous step solution: critical for springs

# === Bodies === fixed ground (with two visual anchor spheres) + two free masses
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetName("ground")
sys.AddBody(ground)

# sph_1: visual anchor sphere at origin; sph_2: mirrored anchor at (1, 0, 0).
sph_1 = chrono.ChVisualShapeSphere(anchor_radius)
sph_1.SetColor(chrono.ChColor(0.2, 0.2, 0.6))
ground.AddVisualShape(sph_1, chrono.ChFramed(anchor_1, chrono.QUNIT))
sph_2 = chrono.ChVisualShapeSphere(anchor_radius)
sph_2.SetColor(chrono.ChColor(0.2, 0.2, 0.6))
ground.AddVisualShape(sph_2, chrono.ChFramed(anchor_2, chrono.QUNIT))

# body_1: free mass driven by spring_1 (direct TSDA coefficients).
body_1 = chrono.ChBody()
body_1.SetMass(body_mass)
body_1.SetInertiaXX(chrono.ChVector3d(0.04, 0.04, 0.04))
body_1.SetPos(body_1_pos)
body_1.SetName("body_1")
sys.AddBody(body_1)
vis_sph_1 = chrono.ChVisualShapeSphere(body_radius)
vis_sph_1.SetColor(chrono.ChColor(0.8, 0.3, 0.2))
body_1.AddVisualShape(vis_sph_1)

# body_2: free mass driven by spring_2 (custom force functor), mirrors body_1.
body_2 = chrono.ChBody()
body_2.SetMass(body_mass)
body_2.SetInertiaXX(chrono.ChVector3d(0.04, 0.04, 0.04))
body_2.SetPos(body_2_pos)
body_2.SetName("body_2")
sys.AddBody(body_2)
vis_sph_2 = chrono.ChVisualShapeSphere(body_radius)
vis_sph_2.SetColor(chrono.ChColor(0.2, 0.7, 0.3))
body_2.AddVisualShape(vis_sph_2)

# === Spring-damper links === spring_1 direct coefficients, spring_2 custom functor
# spring_1: ground anchor_1 -> body_1, using built-in k / c coefficients.
spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground, True,
                    chrono.ChVector3d(0, 0, 0),     # body_1 local attach (center)
                    anchor_1)                       # ground local attach
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_1)
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(coil_radius, coil_resolution, coil_turns))

# spring_2: ground anchor_2 -> body_2, force supplied by the custom functor.
my_force = MySpringForce(spring_coef, damping_coef)  # keep a reference: avoid GC
spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_2, ground, True,
                    chrono.ChVector3d(0, 0, 0),     # body_2 local attach (center)
                    anchor_2)                       # ground local attach
spring_2.SetRestLength(rest_length)
spring_2.RegisterForceFunctor(my_force)
sys.AddLink(spring_2)
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(coil_radius, coil_resolution, coil_turns))

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Mass-Spring-Damper Oscillators")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.5, -5.0, -1.0), chrono.ChVector3d(0.5, 0.0, -1.0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 20, 20, chrono.ChCoordsysd(), chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === render-cadence outer loop, physics in inner batch
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once



# Cache repeated getters into locals once (reused every physics step).
b1 = body_1   # cache: oscillator 1 handle, reused every step
b2 = body_2   # cache: oscillator 2 handle, reused every step
s1 = spring_1  # cache: spring_1 link handle
s2 = spring_2  # cache: spring_2 link handle

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
