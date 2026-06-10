"""Mass-spring-damper chain in PyChrono (NSC system, Irrlicht visualization).

Models a vertical chain of three point masses suspended from a fixed truss by
linear spring-damper links (ChLinkTSDA). The truss is fixed in space; body_1
hangs below it on a spring, body_2 hangs below body_1 on a second spring, and
body_3 hangs below body_2 on a third spring. Each spring has identical rest
length, stiffness and damping. Under gravity the three masses settle into a
stretched static equilibrium while oscillating and decaying toward it — the
classic coupled spring-mass-damper response. No contact/collision is present,
so this is a pure jointed-MBS scene (no collision system).
"""

import os
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Parameters === geometry / physics constants (no bare literals downstream)
time_step = 1e-3                  # integration step (s)
sim_end = 10.0                    # simulation horizon (s)
render_fps = 50.0                 # review render cadence
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

mass_value = 1.0                  # mass of each suspended body (kg)
inertia_xx = 1.0                  # diagonal inertia term (kg m^2)
rest_length = 1.0                 # spring natural length (m)
spring_k = 50.0                   # spring stiffness (N/m)
damping_c = 5.0                   # spring damping (N s/m)

# Vertical chain layout: truss at top, masses spaced one rest length apart.
truss_pos = chrono.ChVector3d(0, 0, 0)
body1_pos = chrono.ChVector3d(0, -rest_length, 0)
body2_pos = chrono.ChVector3d(0, -2.0 * rest_length, 0)
body3_pos = chrono.ChVector3d(0, -3.0 * rest_length, 0)

# === System & gravity === NSC system, gravity along -Y (matches vertical chain)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Bodies === fixed truss + three suspended masses (visual spheres only)
truss = chrono.ChBody()
truss.SetPos(truss_pos)
truss.SetFixed(True)
truss.AddVisualShape(chrono.ChVisualShapeBox(0.4, 0.1, 0.4))
sys.AddBody(truss)

body_1 = chrono.ChBody()
body_1.SetMass(mass_value)
body_1.SetInertiaXX(chrono.ChVector3d(inertia_xx, inertia_xx, inertia_xx))
body_1.SetPos(body1_pos)
body_1.AddVisualShape(chrono.ChVisualShapeSphere(0.15))
sys.AddBody(body_1)

body_2 = chrono.ChBody()
body_2.SetMass(mass_value)
body_2.SetInertiaXX(chrono.ChVector3d(inertia_xx, inertia_xx, inertia_xx))
body_2.SetPos(body2_pos)
body_2.AddVisualShape(chrono.ChVisualShapeSphere(0.15))
sys.AddBody(body_2)

body_3 = chrono.ChBody()
body_3.SetMass(mass_value)
body_3.SetInertiaXX(chrono.ChVector3d(inertia_xx, inertia_xx, inertia_xx))
body_3.SetPos(body3_pos)
body_3.AddVisualShape(chrono.ChVisualShapeSphere(0.15))
sys.AddBody(body_3)

# === Springs === three spring-damper links chaining truss -> b1 -> b2 -> b3
spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(truss, body_1, True,
                    chrono.ChVector3d(0, 0, 0),
                    chrono.ChVector3d(0, 0, 0))
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_k)
spring_1.SetDampingCoefficient(damping_c)
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))
sys.AddLink(spring_1)

spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_1, body_2, True,
                    chrono.ChVector3d(0, 0, 0),
                    chrono.ChVector3d(0, 0, 0))
spring_2.SetRestLength(rest_length)
spring_2.SetSpringCoefficient(spring_k)
spring_2.SetDampingCoefficient(damping_c)
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))
sys.AddLink(spring_2)

spring_3 = chrono.ChLinkTSDA()
spring_3.Initialize(body_2, body_3, True,
                    chrono.ChVector3d(0, 0, 0),
                    chrono.ChVector3d(0, 0, 0))
spring_3.SetRestLength(rest_length)
spring_3.SetSpringCoefficient(spring_k)
spring_3.SetDampingCoefficient(damping_c)
spring_3.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))
sys.AddLink(spring_3)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Mass-Spring-Damper Chain")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # gravity along -Y
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -1.5, 4), chrono.ChVector3d(0, -1.5, 0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, -1.5, 0),
                               chrono.QuatFromAngleX(chrono.CH_PI_2)),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === real-time render-cadence loop; physics advanced in batches

try:
    frame = 0
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
