"""Random spherical particle cluster with mutual attraction in an NSC system.

The simulation creates several collision-enabled spherical particles with
ChRandomShapeCreatorSpheres using the requested Zhang diameter distribution and
constant density.  The bodies attract each other through an explicit pairwise
Newtonian force, and each physics step prints kinetic, potential, and total
energy for the particle system.
"""

import itertools
import contextlib
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants === particle count, timing, and placement kept explicit for review
NUM_PARTICLES = 8
TIME_STEP = 0.005
SIM_END = 4.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
INITIAL_RADIUS = 1.6
INITIAL_SPEED = 0.08
MIN_ATTRACT_DISTANCE = 0.25


# === System & material === zero external gravity isolates the pairwise attraction
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(80)


# === Particle creator === random sphere creator and distributions requested by the prompt
mcreator_spheres = chrono.ChRandomShapeCreatorSpheres()
mcreator_spheres.SetDiameterDistribution(chrono.ChZhangDistribution(0.6, 0.23))
mcreator_spheres.SetDensityDistribution(chrono.ChConstantDistribution(1600))
mcreator_spheres.SetAddCollisionShape(True)
mcreator_spheres.SetAddVisualizationAsset(True)

particles = []
for index in range(NUM_PARTICLES):
    angle = 2.0 * math.pi * index / NUM_PARTICLES
    x_pos = INITIAL_RADIUS * math.cos(angle)
    y_pos = INITIAL_RADIUS * math.sin(angle)
    z_pos = 0.15 * ((index % 3) - 1)
    coords = chrono.ChCoordsysd(chrono.ChVector3d(x_pos, y_pos, z_pos), chrono.QUNIT)
    body = mcreator_spheres.RandomGenerate(coords)
    body.SetName(f"sphere_particle_{index}")
    body.SetPosDt(
        chrono.ChVector3d(
            -INITIAL_SPEED * math.sin(angle),
            INITIAL_SPEED * math.cos(angle),
            0.0,
        )
    )
    sys.AddBody(body)
    particles.append(body)

particle_pairs = list(itertools.combinations(particles, 2))  # cache: reused every step


# === Visualization === Irrlicht window initialized before adding scene elements
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Spherical Particle Energy Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -8, 5), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(
    0.5,
    0.5,
    20,
    20,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -0.8), chrono.QUNIT),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Energy and force update === compute requested diagnostics before each step
def update_attraction_and_print_energy(bodies, pairs):
    for body in bodies:
        body.EmptyAccumulators()

    kinetic_energy = 0.0
    for body in bodies:
        mass = body.GetMass()
        velocity = body.GetPosDt()
        kinetic_energy += 0.5 * mass * velocity.Length2()

    G_constant = 0.03
    potential_energy = 0.0
    for body_a, body_b in pairs:
        pos_a = body_a.GetPos()
        pos_b = body_b.GetPos()
        delta = pos_b - pos_a
        distance = max(delta.Length(), MIN_ATTRACT_DISTANCE)
        mass_a = body_a.GetMass()
        mass_b = body_b.GetMass()
        potential_energy += -G_constant * (mass_a * mass_b) / distance

        force_scale = G_constant * mass_a * mass_b / (distance * distance * distance)
        force = delta * force_scale
        body_a.AccumulateForce(force, pos_a, False)
        body_b.AccumulateForce(-force, pos_b, False)

    total_energy = kinetic_energy + potential_energy
    print(
        f"time={sys.GetChTime():.4f} kinetic={kinetic_energy:.8f} "
        f"potential={potential_energy:.8f} total={total_energy:.8f}"
    )
    return kinetic_energy, potential_energy, total_energy


# === Main loop === render at frame cadence and advance particles with pairwise forces
frame = 0

try:
    with contextlib.ExitStack() as stack:
        while vis.Run() and sys.GetChTime() < SIM_END:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            for _ in range(RENDER_EVERY):
                kinetic, potential, total = update_attraction_and_print_energy(
                    particles, particle_pairs
                )
                sys.DoStepDynamics(TIME_STEP)
                if sys.GetChTime() >= SIM_END:
                    break
except (RuntimeError, ValueError) as exc:
    print(f"Simulation stopped by solver or invalid particle state: {exc}")
    raise
except (OSError, IOError) as exc:
    print(f"Simulation output failed: {exc}")
    raise
finally:
    print(f"Finished particle simulation at t={sys.GetChTime():.4f}")
