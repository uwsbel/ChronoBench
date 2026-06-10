"""Single mass-spring-damper system in PyChrono.

Models one dynamic mass connected to a fixed ground anchor by a linear
spring-damper element (`ChLinkTSDA`). The system is `ChSystemNSC`. Gravity
acts along -Z, so the mass is pulled down, stretches the spring past its rest
length, and the spring-damper restoring + damping forces drive a decaying
vertical oscillation toward a static equilibrium where spring force balances
weight. Bodies: a fixed ground anchor and a single dynamic mass; the link is a
`ChLinkTSDA` carrying a `ChVisualShapeSpring`. This is a pure jointed MBS scene
with no contact, so no collision system is configured. Expected behavior: the
mass bobs vertically along the spring axis with the amplitude shrinking over
time as the damper dissipates energy.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / physics parameters (no bare literals downstream)
time_step = 1e-3          # physics step [s]
sim_end = 10.0            # simulation duration [s]
render_fps = 50.0         # review render cadence [frames/s]

mass_value = 1.0          # mass of the moving body [kg]
mass_radius = 0.15        # visual sphere radius of the mass [m]

anchor_z = 2.0            # fixed ground-anchor height [m]
rest_length = 1.0         # spring natural (unstretched) length [m]
spring_k = 50.0           # linear spring coefficient [N/m]
damping_c = 2.0           # linear damping coefficient [N·s/m]

# Start the mass at the rest length below the anchor so the spring begins
# unstretched and gravity initiates the oscillation.
mass_start_z = anchor_z - rest_length   # precomputed once: initial mass height [m]

coil_radius = 0.08        # spring coil visual radius [m]
coil_resolution = 80      # spring coil visual segment count
coil_turns = 10.0         # spring coil visual turn count

# === System & gravity === single NSC system, gravity along -Z (Z-up world)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# Spring-damper chains are stiff; PSOR + warm start keeps the solver stable.
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
solver = sys.GetSolver().AsIterative()
solver.SetMaxIterations(100)
solver.SetTolerance(1e-10)
solver.EnableWarmStart(True)   # reuse previous step's solution for spring convergence

# === Bodies === fixed ground anchor + single dynamic mass
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0, 0, anchor_z))
ground.EnableCollision(False)
anchor_marker = chrono.ChVisualShapeSphere(0.06)   # small marker at the anchor point
anchor_marker.SetColor(chrono.ChColor(0.2, 0.2, 0.2))
ground.AddVisualShape(anchor_marker)
sys.AddBody(ground)

mass = chrono.ChBody()
mass.SetMass(mass_value)
# Solid-sphere inertia I = (2/5) m r^2 about every axis.
mass_inertia = (2.0 / 5.0) * mass_value * mass_radius * mass_radius
mass.SetInertiaXX(chrono.ChVector3d(mass_inertia, mass_inertia, mass_inertia))
mass.SetPos(chrono.ChVector3d(0, 0, mass_start_z))
mass.EnableCollision(False)
mass_visual = chrono.ChVisualShapeSphere(mass_radius)
mass_visual.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
mass.AddVisualShape(mass_visual)
sys.AddBody(mass)

# === Spring-damper link === ChLinkTSDA between ground anchor and mass
spring = chrono.ChLinkTSDA()
spring.Initialize(ground, mass, True,
                  chrono.ChVector3d(0, 0, 0),    # attachment on ground (local origin)
                  chrono.ChVector3d(0, 0, 0))    # attachment on mass (local origin)
spring.SetRestLength(rest_length)
spring.SetSpringCoefficient(spring_k)
spring.SetDampingCoefficient(damping_c)
sys.AddLink(spring)

# Visual shape belongs to the LINK itself, not to a body.
spring.AddVisualShape(chrono.ChVisualShapeSpring(coil_radius, coil_resolution, coil_turns))

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Mass-Spring-Damper")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, -4, 1.5), chrono.ChVector3d(0, 0, 1.0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === render once per frame, step physics in an inner batch
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once



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
except (RuntimeError, ValueError) as exc:   # solver divergence / bad simulation state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
