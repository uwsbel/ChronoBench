"""Three-body gravitational particle simulation (PyChrono, NSC, Irrlicht).

Models three spherical bodies that attract one another through mutual Newtonian
gravity. Each body is a ChBodyEasySphere; world gravity is disabled and the only
forces are the pairwise inverse-square attractions, re-applied every step via the
per-body force accumulators (EmptyAccumulators + AccumulateForce). The spheres are
given distinct initial positions and velocities so they follow curved, coupled
trajectories — the classic chaotic three-body dance. System type: NSC. Expected
behavior: the three spheres orbit/scatter under mutual attraction with no fall-through
and continuous, bounded motion.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / physics constants and derived initial state
TIME_STEP = 1e-3          # integration step [s]
SIM_END = 20.0            # simulation duration [s]
RENDER_FPS = 50.0         # review render cadence [frames/s]
SPHERE_RADIUS = 1.0       # visual/physical sphere radius [m]
SPHERE_DENSITY = 1000.0   # density [kg/m^3] -> mass from radius
G_CONST = 6.674e-1        # scaled gravitational constant (visible dynamics) [N*m^2/kg^2]
SOFTENING = 0.5           # Plummer softening length to avoid singular forces [m]

# Initial positions for the three-body problem.
INIT_POS = [
    chrono.ChVector3d(10.0, 10.0, 0.0),     # Sphere 1
    chrono.ChVector3d(-10.0, -10.0, 0.0),   # Sphere 2
    chrono.ChVector3d(0.0, 20.0, 0.0),      # Sphere 3
]
# Initial velocities driving the dynamic interaction.
INIT_VEL = [
    chrono.ChVector3d(0.5, 0.0, 0.1),       # Sphere 1
    chrono.ChVector3d(-0.5, 0.0, -0.1),     # Sphere 2
    chrono.ChVector3d(0.0, -0.5, 0.2),      # Sphere 3
]
SPHERE_COLORS = [
    chrono.ChColor(0.9, 0.2, 0.2),
    chrono.ChColor(0.2, 0.9, 0.2),
    chrono.ChColor(0.2, 0.4, 0.9),
]

RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once

# === System & gravity === NSC world with mutual-gravity-only dynamics (no world g)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))   # bodies attract only each other
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Bodies === three gravitating spheres with distinct initial pos/vel
spheres = []
for i in range(3):
    sph = chrono.ChBodyEasySphere(SPHERE_RADIUS, SPHERE_DENSITY, True, False)
    sph.SetPos(INIT_POS[i])
    sph.SetLinVel(INIT_VEL[i])
    sph.GetVisualShape(0).SetColor(SPHERE_COLORS[i])
    sys.AddBody(sph)
    spheres.append(sph)

masses = [s.GetMass() for s in spheres]   # cache: fetched once, reused every step


def apply_gravity():
    """Accumulate pairwise Newtonian attraction onto every sphere for this step."""
    for s in spheres:
        s.EmptyAccumulators()
    positions = [s.GetPos() for s in spheres]
    for a in range(3):
        for b in range(a + 1, 3):
            d = positions[b] - positions[a]
            dist2 = d.Length2() + SOFTENING * SOFTENING
            dist = math.sqrt(dist2)
            mag = G_CONST * masses[a] * masses[b] / dist2
            force = d * (mag / dist)                 # a pulled toward b
            spheres[a].AccumulateForce(force, positions[a], False)
            spheres[b].AccumulateForce(-force, positions[b], False)


# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Three-Body Gravitational Particles")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -60, 30), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(2.0, 2.0, 40, 40, chrono.ChCoordsysd(), chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === step physics under mutual gravity; capture review video/CSV

try:
    frame = 0
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            apply_gravity()
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
