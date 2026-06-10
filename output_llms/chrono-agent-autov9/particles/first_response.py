"""Gravitational N-body particle attraction with a PyChrono particle emitter.

Model
-----
A `ChParticleEmitter` continuously spawns rigid bodies of RANDOM shape (spheres,
boxes, cylinders chosen by a family creator), at RANDOM positions over a
rectangular outlet, with RANDOM velocities (any direction) and RANDOM
orientations (uniform alignment). Every spawned body is collected through an
add-body callback into a registry.

Each simulation step a CUSTOM gravitational force is applied between every pair
of emitted bodies (Newtonian inverse-square attraction, softened to avoid a
singularity at small separation). World gravity is disabled so the ONLY force
driving the dynamics is this mutual attraction — particles drift, accelerate
toward the evolving center of mass, and cluster.

System type
-----------
`ChSystemNSC` (rigid bodies; Bullet collision enabled so colliding particles
resolve as hard contacts). Visualized with Irrlicht.

Expected behavior
-----------------
The particle count rises as the emitter fires; the cloud, under mutual
attraction, contracts toward its center of mass and forms a clump. Logged to
`simulation_data.csv` (aggregate per step) and `particles.csv` (per-particle
final snapshot); a timeseries plot is written to `simulation_timeseries.png`.
"""

import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / physics / emitter / loop parameters
TIME_STEP = 5e-3                 # integration step [s]
SIM_END = 8.0                    # total simulated time [s]
RENDER_FPS = 30.0                # review-video frame rate [frames/s]

EMIT_PARTICLES_PER_SECOND = 150.0  # emitter flow rate [1/s] (front-loads the cloud fast)
EMIT_RESERVOIR = 150             # cap on total emitted particles
OUTLET_WIDTH = 1.2               # emitter outlet extent along local X [m]
OUTLET_HEIGHT = 1.2             # emitter outlet extent along local Y [m]
OUTLET_Z = 5.0                   # outlet height above origin [m] (clump forms in mid-air)
VEL_MIN = 0.0                    # min initial speed modulus [m/s]
VEL_MAX = 0.10                   # max initial speed modulus [m/s] (low: attraction dominates)

SPHERE_DIA_MIN = 0.18            # random sphere diameter range [m]
SPHERE_DIA_MAX = 0.34
BOX_SIZE_MIN = 0.16              # random box X-size range [m]
BOX_SIZE_MAX = 0.30
CYL_DIA_MIN = 0.16              # random cylinder diameter range [m]
CYL_DIA_MAX = 0.28
PART_DENSITY = 1000.0            # particle material density [kg/m3]

GRAV_CONST = 0.015               # tuned attraction constant (NOT physical 6.67e-11)
SOFTENING = 0.8                  # Plummer softening length [m] (avoids 1/r^2 slingshot)
DAMPING = 0.25                   # linear velocity damping [1/s] (dissipates KE so the
                                 # cloud settles into a bound clump instead of ejecting)

FRICTION = 0.3                   # contact friction
RESTITUTION = 0.1                # contact restitution

# Headless validation gate: a fast, windowless physics check (no Irrlicht window).
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))

# === Derived constants === computed ONCE before the loop (never recomputed inside)
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END           # short check when validating
soft_sq = SOFTENING * SOFTENING                                # precomputed once

# === System & gravity === NSC rigid-body world; world gravity OFF so the only
# driver is the custom pairwise attraction applied each step.
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))   # disable world gravity
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(60)

# Shared NSC contact material for every emitted particle.
particle_mat = chrono.ChContactMaterialNSC()
particle_mat.SetFriction(FRICTION)
particle_mat.SetRestitution(RESTITUTION)

# === Bodies === particle emitter producing random-shape rigid bodies.
# Registry of every emitted body, filled by the add-body callback below; the
# custom gravity loop iterates over it each step.
emitted_bodies = []


class ParticleCollector(chrono.ChRandomShapeCreator_AddBodyCallback):
    """Callback fired for every body the emitter creates.

    Records the body and gives it a force accumulator slot so the main loop can
    inject the custom gravitational force without recreating ChForce objects.
    """

    def __init__(self, material):
        super().__init__()
        self.material = material   # cache: contact material reused for every particle

    def OnAddBody(self, body, coords, creator):
        body.GetCollisionModel().SetAllShapesMaterial(self.material)
        body.EnableCollision(True)
        acc_idx = body.AddAccumulator()           # one accumulator per body, reused each step
        body.SetTag(acc_idx)                       # stash accumulator index on the body
        emitted_bodies.append(body)


# Emitter flow control: spawn at a fixed particle rate up to a reservoir cap.
emitter = chrono.ChParticleEmitter()
emitter.SetParticlesPerSecond(EMIT_PARTICLES_PER_SECOND)
emitter.SetFlowControlMode(chrono.ChParticleEmitter.FLOW_PARTICLESPERSECOND)
emitter.SetUseParticleReservoir(True)
emitter.SetParticleReservoirAmount(EMIT_RESERVOIR)

# Random POSITIONS spread over a thin rectangular outlet box. The positioner
# samples in the box's LOCAL frame (centered at origin); the emission is lifted
# to OUTLET_Z by the pre-transform passed to EmitParticles below.
outlet_box = chrono.ChBox(OUTLET_WIDTH, OUTLET_HEIGHT, 0.05)   # flat rectangular outlet
positioner = chrono.ChRandomParticlePositionOnGeometry()
positioner.SetGeometry(outlet_box, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))
emitter.SetParticlePositioner(positioner)

# Pre-transform applied to every emitted particle: lifts the outlet to OUTLET_Z so
# the cloud forms in mid-air above the grid (precomputed once, reused every step).
emit_pre_transform = chrono.ChFrameMovingd(chrono.ChVector3d(0, 0, OUTLET_Z), chrono.QUNIT)

# Random VELOCITIES: any direction, modulus uniform in [VEL_MIN, VEL_MAX].
velocities = chrono.ChRandomParticleVelocityAnyDirection()
velocities.SetModulusDistribution(chrono.ChUniformDistribution(VEL_MIN, VEL_MAX))
emitter.SetParticleVelocity(velocities)

# Random ORIENTATIONS: uniform alignment over the sphere of rotations.
emitter.SetParticleAligner(chrono.ChRandomParticleAlignmentUniform())

# Random SHAPES: a family creator mixing spheres, boxes and cylinders so each
# emitted body has a randomly chosen shape and size.
sphere_creator = chrono.ChRandomShapeCreatorSpheres()
sphere_creator.SetDiameterDistribution(chrono.ChUniformDistribution(SPHERE_DIA_MIN, SPHERE_DIA_MAX))
sphere_creator.SetDensityDistribution(chrono.ChConstantDistribution(PART_DENSITY))

box_creator = chrono.ChRandomShapeCreatorBoxes()
box_creator.SetXsizeDistribution(chrono.ChUniformDistribution(BOX_SIZE_MIN, BOX_SIZE_MAX))
box_creator.SetSizeRatioZDistribution(chrono.ChUniformDistribution(0.7, 1.3))
box_creator.SetSizeRatioYZDistribution(chrono.ChUniformDistribution(0.7, 1.3))
box_creator.SetDensityDistribution(chrono.ChConstantDistribution(PART_DENSITY))

cyl_creator = chrono.ChRandomShapeCreatorCylinders()
cyl_creator.SetDiameterDistribution(chrono.ChUniformDistribution(CYL_DIA_MIN, CYL_DIA_MAX))
cyl_creator.SetLengthFactorDistribution(chrono.ChUniformDistribution(0.8, 1.6))
cyl_creator.SetDensityDistribution(chrono.ChConstantDistribution(PART_DENSITY))

shape_creator = chrono.ChRandomShapeCreatorFromFamilies()
shape_creator.AddFamily(sphere_creator, 0.34)
shape_creator.AddFamily(box_creator, 0.33)
shape_creator.AddFamily(cyl_creator, 0.33)
shape_creator.SetAddCollisionShape(True)
shape_creator.SetAddVisualizationAsset(True)
emitter.SetParticleCreator(shape_creator)

# Register the collector on the emitter so every spawned body is captured.
collector = ParticleCollector(particle_mat)
emitter.RegisterAddBodyCallback(collector)


def apply_mutual_gravity():
    """Apply softened pairwise Newtonian attraction across all emitted bodies.

    Force on i from j: F = G * m_i * m_j * r_hat / (|r|^2 + eps^2).
    Uses each body's pre-allocated accumulator (emptied first) so no per-step
    ChForce allocation occurs.
    """
    n = len(emitted_bodies)
    if n < 2:
        # Still clear accumulators of any lone body to keep state consistent.
        for b in emitted_bodies:
            b.EmptyAccumulator(b.GetTag())
        return
    # Snapshot positions / masses / velocities once per call (cache: avoids repeated getters).
    pos = [b.GetPos() for b in emitted_bodies]
    mass = [b.GetMass() for b in emitted_bodies]
    vel = [b.GetPosDt() for b in emitted_bodies]
    for b in emitted_bodies:
        b.EmptyAccumulator(b.GetTag())
    for i in range(n):
        pi = pos[i]
        mi = mass[i]
        fx = fy = fz = 0.0
        for j in range(n):
            if i == j:
                continue
            dx = pos[j].x - pi.x
            dy = pos[j].y - pi.y
            dz = pos[j].z - pi.z
            r2 = dx * dx + dy * dy + dz * dz + soft_sq
            inv_r = 1.0 / math.sqrt(r2)
            mag = GRAV_CONST * mi * mass[j] / r2
            fx += mag * dx * inv_r
            fy += mag * dy * inv_r
            fz += mag * dz * inv_r
        # Linear damping bleeds kinetic energy so the cloud stays a bound clump
        # instead of slingshotting particles outward (F_damp = -DAMPING * m * v).
        vi = vel[i]
        fx -= DAMPING * mi * vi.x
        fy -= DAMPING * mi * vi.y
        fz -= DAMPING * mi * vi.z
        bi = emitted_bodies[i]
        bi.AccumulateForce(bi.GetTag(), chrono.ChVector3d(fx, fy, fz), pi, False)


def cloud_stats():
    """Return (count, center_of_mass, rms_radius, mean_speed) for the cloud."""
    n = len(emitted_bodies)
    if n == 0:
        return 0, chrono.ChVector3d(0, 0, 0), 0.0, 0.0
    total_m = 0.0
    cx = cy = cz = 0.0
    for b in emitted_bodies:
        m = b.GetMass()
        p = b.GetPos()
        total_m += m
        cx += m * p.x
        cy += m * p.y
        cz += m * p.z
    com = chrono.ChVector3d(cx / total_m, cy / total_m, cz / total_m)
    rss = 0.0
    speed_sum = 0.0
    for b in emitted_bodies:
        p = b.GetPos()
        v = b.GetPosDt()
        rss += (p.x - com.x) ** 2 + (p.y - com.y) ** 2 + (p.z - com.z) ** 2
        speed_sum += math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
    rms_r = math.sqrt(rss / n)
    return n, com, rms_r, speed_sum / n


# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
if not HEADLESS:
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)   # Z-up world
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Gravitational Particle Attraction (Emitter)")
    vis.Initialize()                                     # Initialize FIRST (Irrlicht)
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(5.5, -5.5, 6.0), chrono.ChVector3d(0, 0, OUTLET_Z))
    vis.AddTypicalLights()
    vis.AddGrid(0.5, 0.5, 40, 40,
                chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                chrono.ChColor(0.4, 0.4, 0.4))           # ground reference grid

# === Main loop === emit + custom gravity + step; log aggregate stats each step.
os.makedirs("frames", exist_ok=True)   # guard against missing output dir

data_time = []
data_count = []
data_rms = []
data_speed = []

data_file = None
try:
    try:
        data_file = open("simulation_data.csv", "w", newline="")   # may fail on disk/perm
    except (OSError, IOError) as exc:    # disk full / permission denied
        print("Could not open simulation_data.csv:", exc)
        raise
    writer = csv.writer(data_file)
    writer.writerow(["time", "num_particles", "com_x", "com_y", "com_z",
                     "rms_radius", "mean_speed"])

    frame = 0
    bound_count = 0   # how many emitted bodies are already registered with Irrlicht
    while (HEADLESS or vis.Run()) and sys.GetChTime() < run_end:
        if not HEADLESS:
            # Bodies the emitter adds AFTER Initialize() must be bound to the
            # Irrlicht scene or they render invisibly; rebind only when count grew.
            if len(emitted_bodies) != bound_count:
                vis.BindAll()
                bound_count = len(emitted_bodies)
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
            frame += 1
        for _ in range(render_every):
            t = sys.GetChTime()                       # cache: one call per step
            emitter.EmitParticles(sys, TIME_STEP, emit_pre_transform)  # spawn at OUTLET_Z
            apply_mutual_gravity()                    # custom pairwise attraction
            n, com, rms_r, mean_v = cloud_stats()
            writer.writerow([t, n, com.x, com.y, com.z, rms_r, mean_v])
            data_time.append(t)
            data_count.append(n)
            data_rms.append(rms_r)
            data_speed.append(mean_v)
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= run_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    # Flush + close the CSV even if a step diverged mid-run.
    if data_file is not None:
        data_file.flush()
        data_file.close()

# === Post-processing === per-particle snapshot + timeseries plot from the log.
with open("particles.csv", "w", newline="") as pf:
    pwriter = csv.writer(pf)
    pwriter.writerow(["index", "mass", "pos_x", "pos_y", "pos_z",
                      "vel_x", "vel_y", "vel_z"])
    for idx, b in enumerate(emitted_bodies):
        p = b.GetPos()
        v = b.GetPosDt()
        pwriter.writerow([idx, b.GetMass(), p.x, p.y, p.z, v.x, v.y, v.z])

print(f"Emitted {len(emitted_bodies)} particles; logged {len(data_time)} steps.")

if data_time:
    t_arr = np.array(data_time)
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(8, 9))
    ax1.plot(t_arr, data_count, color="tab:blue")
    ax1.set(ylabel="particle count")
    ax1.grid(True)
    ax2.plot(t_arr, data_rms, color="tab:green")
    ax2.set(ylabel="RMS radius [m]")
    ax2.grid(True)
    ax3.plot(t_arr, data_speed, color="tab:red")
    ax3.set(ylabel="mean speed [m/s]", xlabel="time [s]")
    ax3.grid(True)
    fig.suptitle("Gravitational particle attraction")
    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=110)
    plt.close(fig)
