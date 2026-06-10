"""Simple pendulum simulation (PyChrono, NSC system, Irrlicht visualization).

Models a single rigid pendulum hinged to a fixed ground body by a revolute
joint. The system is a pure jointed multi-body mechanism with no contact, so no
collision system is configured. Gravity acts along -Z (Z-up world); the
pendulum, offset from its pivot, swings under gravity about the hinge axis.
The simulation renders with Irrlicht and periodically logs the pendulum body's
position and velocity. Expected behavior: the pendulum oscillates about the
fixed pivot like a physical swinging arm.
"""

import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / physics constants and derived pivot geometry
time_step = 1e-3            # integration step [s]
sim_end = 10.0             # simulation duration [s]
render_fps = 50.0         # review-video frame cadence [frames/s]
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

pend_mass = 1.0            # pendulum mass [kg]
pivot_pos = chrono.ChVector3d(0, 0, 1)   # fixed hinge location (world)
pend_pos = chrono.ChVector3d(1, 0, 1)    # pendulum COM, offset +X from pivot

# === System & gravity === NSC multibody system, gravity along -Z (Z-up)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# === Bodies === fixed ground anchor + the swinging pendulum body
ground = chrono.ChBody()
ground.SetFixed(True)
sys.AddBody(ground)

pend = chrono.ChBody()
pend.SetMass(pend_mass)
pend.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))
pend.SetPos(pend_pos)
sys.AddBody(pend)

# Visual rod for the pendulum arm (cylinder along body-local X, pivot -> COM).
pend_rod = chrono.ChVisualShapeCylinder(0.05, 2.0)
pend_rod.SetColor(chrono.ChColor(0.6, 0.2, 0.2))
pend.AddVisualShape(pend_rod, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# === Joints / constraints === revolute hinge: pendulum to ground at the pivot
# Hinge axis is world +Y (normal to the XZ swing plane); map joint local +Z to +Y.
q_hinge = chrono.QuatFromAngleAxis(chrono.CH_PI_2, chrono.VECT_X)
hinge = chrono.ChLinkLockRevolute()
hinge.Initialize(pend, ground, chrono.ChFramed(pivot_pos, q_hinge))
sys.AddLink(hinge)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Simple Pendulum")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2, -4, 2), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === render at fixed cadence, advance physics, log pose/velocity
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
