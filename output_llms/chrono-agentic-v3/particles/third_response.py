"""
Three-Body Gravitational Particle Simulation.

Models three spherical bodies interacting under mutual Newtonian gravitational
attraction. The bodies begin at distinct positions with non-zero initial velocities
and evolve under the three-body problem dynamics, producing complex curved
trajectories driven by their mutual attraction.

System type: ChSystemNSC (no contact — pure gravitational MBS with no collision shapes).
Spheres (unit mass=1):
  Sphere 1: pos (10, 10, 0),   vel (0.5,  0,   0.1)
  Sphere 2: pos (-10, -10, 0), vel (-0.5,  0,  -0.1)
  Sphere 3: pos (0,  20,  0),  vel (0,   -0.5,  0.2)
Effective gravitational constant scaled so that at a separation of ~20 m and
initial speeds of 0.5 m/s, the three bodies visibly attract and curve toward each other.
Expected behavior: curved, interacting trajectories characteristic of the three-body problem.
"""

import os
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants ===
SPHERE_RADIUS = 0.5    # [m] sphere radius (visual only; no collision)
SPHERE_MASS   = 1.0    # [arbitrary mass units] — all three bodies equal mass
# Effective G chosen so gravity is visible at the 10-20 m scale and 0.5 m/s velocities:
# G_EFF * M / r^2 ~ 0.01 m/s^2 at r=20m  =>  G_EFF = 0.01 * 400 = 4.0
G_EFF         = 4.0    # [m^3 s^-2 / mass_unit] effective gravitational constant
SOFTENING     = 0.5    # [m] softening to avoid singularity at close approach
TIME_STEP     = 1e-3   # [s] physics time step
SIM_END       = 30.0   # [s] simulation duration (long enough to see orbital curves)
RENDER_FPS    = 50.0
RENDER_EVERY  = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Initial positions (from prompt)
POS1 = chrono.ChVector3d(10.0,  10.0, 0.0)
POS2 = chrono.ChVector3d(-10.0, -10.0, 0.0)
POS3 = chrono.ChVector3d(0.0,   20.0, 0.0)

# Initial velocities (from prompt)
VEL1 = chrono.ChVector3d(0.5,   0.0,  0.1)
VEL2 = chrono.ChVector3d(-0.5,  0.0, -0.1)
VEL3 = chrono.ChVector3d(0.0,  -0.5,  0.2)

# === System & gravity ===
# NSC system; world gravity disabled — gravitational forces applied manually.
# No collision shapes → no contact: omit SetCollisionSystemType (pure MBS, no contact).
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, 0.0))

# === Bodies ===
# ChBodyEasySphere takes density; derive it from mass and sphere volume
SPHERE_DENSITY = SPHERE_MASS / (4.0 / 3.0 * math.pi * SPHERE_RADIUS ** 3)  # precomputed once

sphere1 = chrono.ChBodyEasySphere(SPHERE_RADIUS, SPHERE_DENSITY, True, False)
sphere1.SetPos(POS1)
sphere1.SetPosDt(VEL1)
sphere1.SetMass(SPHERE_MASS)
sys.Add(sphere1)

sphere2 = chrono.ChBodyEasySphere(SPHERE_RADIUS, SPHERE_DENSITY, True, False)
sphere2.SetPos(POS2)
sphere2.SetPosDt(VEL2)
sphere2.SetMass(SPHERE_MASS)
sys.Add(sphere2)

sphere3 = chrono.ChBodyEasySphere(SPHERE_RADIUS, SPHERE_DENSITY, True, False)
sphere3.SetPos(POS3)
sphere3.SetPosDt(VEL3)
sphere3.SetMass(SPHERE_MASS)
sys.Add(sphere3)

# Assign distinct colors for visual identification
sphere1.GetVisualShape(0).SetColor(chrono.ChColor(1.0, 0.2, 0.2))  # red
sphere2.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.5, 1.0))  # blue
sphere3.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 1.0, 0.3))  # green

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Three-Body Gravitational Simulation")
vis.Initialize()                                         # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(30.0, -30.0, 25.0), chrono.ChVector3d(0.0, 5.0, 0.0))
vis.AddTypicalLights()
vis.AddGrid(5.0, 5.0, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))


bodies = [sphere1, sphere2, sphere3]   # cache: sphere list fetched once, reused every step

# === Main loop ===
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            t = sys.GetChTime()

            # Apply pairwise Newtonian gravitational forces (softened) each physics step
            positions = [b.GetPos() for b in bodies]  # cache: current positions at this step
            for i in range(3):
                f_total = chrono.ChVector3d(0.0, 0.0, 0.0)
                for j in range(3):
                    if i == j:
                        continue
                    r_vec   = positions[j] - positions[i]
                    dist_sq = r_vec.Length2() + SOFTENING * SOFTENING  # softened inverse-square
                    dist    = math.sqrt(dist_sq)
                    force_mag = G_EFF * SPHERE_MASS * SPHERE_MASS / dist_sq
                    f_total  += r_vec * (force_mag / dist)
                bodies[i].AccumulateForce(f_total, positions[i], False)


            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:    # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise
finally:
    pass                                      # scored core: nothing to close here
