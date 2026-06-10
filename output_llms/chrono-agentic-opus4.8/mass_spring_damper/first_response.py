"""Single mass-spring-damper system in PyChrono (NSC, no contact).

Models one rigid mass connected to a fixed ground anchor by a translational
spring-damper (`ChLinkTSDA`) using default linear spring/damper coefficients.
The mass is offset from the spring's rest length so it oscillates vertically
under gravity and the elastic/damping force, settling toward equilibrium.
Bodies: a fixed ground reference and one dynamic cube mass. Visualization is
Irrlicht with a coil visual on the spring and a box visual on the mass.
Expected behavior: damped vertical oscillation of the mass about equilibrium.
"""

import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants === geometry / physics / timing (no bare literals downstream)
time_step = 1e-3
sim_end = 5.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))        # precomputed once

mass_value = 1.0                 # mass of the oscillating body [kg]
mass_size = 0.4                  # cube edge length [m]
anchor_pos = chrono.ChVector3d(0, 1.0, 0)    # fixed spring upper attachment
mass_pos = chrono.ChVector3d(0, 0.0, 0)      # initial mass position (stretched)

# === System & gravity === NSC system, gravity along -Y; pure jointed MBS (no contact)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Bodies === fixed ground anchor + one dynamic cube mass
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(anchor_pos)
sys.AddBody(ground)
ground_vis = chrono.ChVisualShapeSphere(0.05)
ground_vis.SetColor(chrono.ChColor(0.2, 0.2, 0.2))
ground.AddVisualShape(ground_vis)

body = chrono.ChBody()
body.SetMass(mass_value)
body.SetInertiaXX(chrono.ChVector3d(0.04, 0.04, 0.04))
body.SetPos(mass_pos)
sys.AddBody(body)
mass_vis = chrono.ChVisualShapeBox(mass_size, mass_size, mass_size)
mass_vis.SetColor(chrono.ChColor(0.6, 0.2, 0.2))
body.AddVisualShape(mass_vis)

# === Joints / constraints === translational spring-damper (default linear params)
spring = chrono.ChLinkTSDA()
spring.Initialize(body, ground, True,
                  chrono.ChVector3d(0, 0, 0),
                  chrono.ChVector3d(0, 0, 0))
spring.SetRestLength(0.5)
spring.SetSpringCoefficient(50.0)
spring.SetDampingCoefficient(5.0)
sys.AddLink(spring)
spring.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Mass-Spring-Damper")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.5, 0.5, 2.5), chrono.ChVector3d(0, 0.2, 0))
vis.AddTypicalLights()
vis.AddGrid(0.2, 0.2, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, -0.6, 0),
                               chrono.QuatFromAngleX(chrono.CH_PI_2)),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === real-time render-cadence loop stepping the spring-damper dynamics
os.makedirs("cam", exist_ok=True)   # guard against missing output dir

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
    import traceback; traceback.print_exc()
    raise
