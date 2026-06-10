"""
Three-Body Particle Simulation using PyChrono (ChSystemNSC).

Models a classic three-body problem with three spherical bodies placed at
large separations and given initial velocities to simulate dynamic interactions.

System: ChSystemNSC, no gravity (free-space three-body dynamics).
Bodies:
  - Sphere 1 (red):    position (0, 0, 0),    velocity (0.5, 0, 0.1)
  - Sphere 2 (green):  position (-10, -10, 0), velocity (-0.5, 0, -0.1)
  - Sphere 3 (blue):   position (0, 20, 0),   velocity (0, -0.5, 0.2)

Expected behavior: all three spheres move under mutual collision forces;
when trajectories bring them close, collision impulses redirect them.
"""

# === Imports ===
import os
import csv
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Simulation parameters ===
time_step    = 5e-3    # physics time step [s]
sim_end      = 30.0    # total simulation duration [s]
render_fps   = 50.0    # frames per second for review video
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# --- Sphere geometry (larger radius for better visibility) ---
sphere_radius  = 1.5     # [m]
sphere_density = 1000.0  # [kg/m^3]

# --- Initial positions ---
pos1 = chrono.ChVector3d(0.0,    0.0,  0.0)
pos2 = chrono.ChVector3d(-10.0, -10.0, 0.0)
pos3 = chrono.ChVector3d(0.0,   20.0,  0.0)

# --- Initial velocities ---
vel1 = chrono.ChVector3d( 0.5, 0.0,  0.1)
vel2 = chrono.ChVector3d(-0.5, 0.0, -0.1)
vel3 = chrono.ChVector3d( 0.0,-0.5,  0.2)

# === System & gravity ===
# NSC system with gravity disabled — free-space three-body dynamics
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(100)

# === Contact material ===
# NSC material with moderate restitution for visible collision bounces
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.3)
mat.SetRestitution(0.6)

# === Bodies ===
# Sphere 1 — red, starts at origin
sphere1 = chrono.ChBodyEasySphere(sphere_radius, sphere_density, True, True, mat)
sphere1.SetPos(pos1)
sphere1.SetPosDt(vel1)
sphere1.GetVisualShape(0).SetColor(chrono.ChColor(0.9, 0.2, 0.2))
sys.Add(sphere1)

# Sphere 2 — green, starts at (-10, -10, 0)
sphere2 = chrono.ChBodyEasySphere(sphere_radius, sphere_density, True, True, mat)
sphere2.SetPos(pos2)
sphere2.SetPosDt(vel2)
sphere2.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.8, 0.2))
sys.Add(sphere2)

# Sphere 3 — blue, starts at (0, 20, 0)
sphere3 = chrono.ChBodyEasySphere(sphere_radius, sphere_density, True, True, mat)
sphere3.SetPos(pos3)
sphere3.SetPosDt(vel3)
sphere3.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.4, 0.9))
sys.Add(sphere3)

# === Visualization ===
# Full Irrlicht block: Initialize() FIRST, scene elements after
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Three-Body Particle Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
# Camera positioned overhead to observe all three bodies spread across ~30 m
vis.AddCamera(
    chrono.ChVector3d(5.0, -60.0, 50.0),
    chrono.ChVector3d(0.0,   5.0,  0.0),
)
vis.AddTypicalLights()
vis.AddGrid(
    5.0, 5.0, 20, 20,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    chrono.ChColor(0.3, 0.3, 0.3),
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
            t = sys.GetChTime()  # cache: fetched once per step, reused below
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
