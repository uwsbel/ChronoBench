"""Three-body gravitational simulation in PyChrono (NSC system, Irrlicht).

Models three spherical point masses ("particles") that attract one another
through mutual Newtonian gravity. The world (uniform) gravity field is disabled
and collisions are turned off: the ONLY forces are the pairwise inverse-square
attractions, applied each physics step through per-body force accumulators.

System type:   ChSystemNSC (rigid bodies, no contact, no world gravity)
Bodies:        sphere_1, sphere_2, sphere_3 (each a ChBodyEasySphere)
Initial state: positions and velocities below place the trio in a bounded,
               continuously interacting orbit-like motion.
Expected:      the three spheres swing around their common centre of mass,
               trading momentum, with no body flying off to infinity over the
               simulated window (a softening length keeps close passes finite).
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants === geometry, masses, gravity model and time-stepping
time_step = 1e-3            # physics step [s]
sim_end = 30.0             # total simulated time [s]
render_fps = 50.0          # review render cadence [frames/s]

SPHERE_RADIUS = 0.8        # visual/collision-free sphere radius [m]
SPHERE_DENSITY = 1000.0    # density [kg/m^3] (mass set explicitly below)
BODY_MASS = 50.0           # point mass of each particle [kg]

G_CONST = 6.0              # tuned gravitational constant for visible, bounded motion
SOFTENING = 1.0            # Plummer softening length [m] — keeps close passes finite

# Initial positions [m] — Sphere 1 starts at the origin, the other two offset.
INIT_POS = [
    chrono.ChVector3d(0.0, 0.0, 0.0),      # sphere 1
    chrono.ChVector3d(-10.0, -10.0, 0.0),  # sphere 2
    chrono.ChVector3d(0.0, 20.0, 0.0),     # sphere 3
]
# Initial velocities [m/s] — set up dynamic, mutually interacting trajectories.
INIT_VEL = [
    chrono.ChVector3d(0.5, 0.0, 0.1),      # sphere 1
    chrono.ChVector3d(-0.5, 0.0, -0.1),    # sphere 2
    chrono.ChVector3d(0.0, -0.5, 0.2),     # sphere 3
]

render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once
soft_sq = SOFTENING * SOFTENING                               # precomputed once


# === System & gravity === NSC system with world gravity OFF (pure N-body forces)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  # only mutual gravity acts
# No SetCollisionSystemType: collisions are disabled, this is a pure-force MBS scene.

# === Bodies === three spherical point masses with prescribed state, collision off
bodies = []
accum_idx = []  # per-body force-accumulator index (created once, reused every step)
for i in range(3):
    sph = chrono.ChBodyEasySphere(SPHERE_RADIUS, SPHERE_DENSITY, True, False)  # visualize, no collide
    sph.SetMass(BODY_MASS)
    sph.SetPos(INIT_POS[i])
    sph.SetPosDt(INIT_VEL[i])
    sph.EnableCollision(False)
    sys.Add(sph)
    accum_idx.append(sph.AddAccumulator())  # cache: accumulator handle, reused every step
    bodies.append(sph)

bodies[0].GetVisualShape(0).SetColor(chrono.ChColor(0.90, 0.30, 0.20))
bodies[1].GetVisualShape(0).SetColor(chrono.ChColor(0.20, 0.70, 0.30))
bodies[2].GetVisualShape(0).SetColor(chrono.ChColor(0.25, 0.45, 0.90))

masses = [b.GetMass() for b in bodies]  # cache: masses fetched once, reused every step


def apply_mutual_gravity():
    """Reset accumulators and load the pairwise softened Newtonian forces."""
    pos = [b.GetPos() for b in bodies]  # cache: poses fetched once per force update
    for i in range(3):
        bodies[i].EmptyAccumulator(accum_idx[i])
    for i in range(3):
        for j in range(i + 1, 3):
            d = pos[j] - pos[i]
            r2 = d.x * d.x + d.y * d.y + d.z * d.z + soft_sq
            inv_r = 1.0 / math.sqrt(r2)
            f_mag = G_CONST * masses[i] * masses[j] * inv_r * inv_r
            f = d * (f_mag * inv_r)  # force on i toward j
            bodies[i].AccumulateForce(accum_idx[i], f, pos[i], False)
            bodies[j].AccumulateForce(accum_idx[j], -f, pos[j], False)


# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Three-Body Gravitational Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 60), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(2.0, 2.0, 40, 40, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === advance N-body dynamics; render once per frame, log each step

try:

    frame = 0
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            apply_mutual_gravity()
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid body state
    import traceback
    traceback.print_exc()
    raise

# === Post-processing === flush data, assemble review video + plot, prune frames
