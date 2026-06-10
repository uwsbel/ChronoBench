"""Gravitational N-body particle cloud (PyChrono 9.0.x, Irrlicht).

This script models a self-gravitating cloud of spherical rigid bodies. The
bodies are produced by a ``ChParticleEmitter`` whose shape creator is a
``ChRandomShapeCreatorSpheres``: sphere diameters are drawn from a Zhang
distribution (average 0.6, minimum 0.23) and the material density is a constant
1600 kg/m^3. World gravity is disabled; instead every ordered pair of bodies
attracts every other body through a Newtonian inverse-square force
``F = G * mA * mB / r^2`` applied each step via per-body force accumulators, so
the cloud slowly contracts under mutual gravitation.

System type: ChSystemNSC (rigid bodies treated as gravitating point masses;
sphere-sphere contact is disabled so the only interaction is the softened
mutual gravity). Main bodies: a set of emitted spheres (no fixed floor — this is
a free-floating gravitational cloud). Expected behavior: the particles drift toward the cloud's
centre of mass; total mechanical energy (kinetic + gravitational potential)
stays approximately bounded while kinetic and potential energy trade off.

Outputs: simulation_data.csv (time + energies + COM), particles.csv (per-body
final state), simulation_timeseries.png (energy vs time), and review frames
under frames/ when run with a window.
"""

import os
import csv
import math
import itertools

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless-safe backend for PNG export
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / physics parameters (no bare literals downstream)
TIME_STEP = 5e-3          # physics step [s]
SIM_END = 6.0             # simulation duration [s]
RENDER_FPS = 30.0         # review-video frame rate [fps]

EMIT_SECONDS = 0.0        # emit the whole cloud once at t=0 (single burst)
PARTICLES_PER_SECOND = 1200.0   # emitter flow rate -> ~burst count over one step
TARGET_PARTICLES = 60     # reservoir cap: total spheres in the cloud

# Sphere shape-creator distributions (FINAL values requested by the task).
ZHANG_AVERAGE_DIAMETER = 0.6    # average sphere diameter [m]
ZHANG_MINIMUM_DIAMETER = 0.23   # minimum sphere diameter [m]
SPHERE_DENSITY = 1600.0         # constant material density [kg/m^3]

# Emission region: spheres seeded over a cubic shell centred at the origin.
SEED_BOX_SIZE = 8.0       # full edge length of the seeding box [m]

# Newtonian gravitational constant (scaled up so attraction is visible at
# laboratory scale within SIM_END; the real 6.674e-11 would be imperceptible).
# Tuned together with SOFTENING so total mechanical energy is conserved across
# the run (the cloud contracts and rebounds without numerical blow-up).
G_CONSTANT = 5e-4         # effective gravitational constant [m^3 kg^-1 s^-2]
SOFTENING = 1.5           # Plummer softening length [m] — bounds the r->0 force

ENABLE_CONTACT = False    # point-mass gravity cloud — sphere-sphere contact OFF
                          # (overlapping spawn positions would otherwise blow up NSC)

# Derived constants (precomputed once — never recompute inside the loop).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
SOFTENING_SQ = SOFTENING * SOFTENING                          # precomputed once
CAMERA_EYE = (7.0, -7.0, 4.0)     # close enough that the ~4 m cloud fills the view
CAMERA_TARGET = (0.0, 0.0, 0.0)
SPHERE_COLOR = (0.95, 0.55, 0.15)  # warm colour so the spheres read clearly on screen

HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))  # fast, windowless validation run

# === System & gravity === NSC rigid-body system; world gravity OFF (mutual gravity only)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  # attraction comes from accumulators
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(80)

# === Contact material === NSC material kept for completeness; contact stays OFF
# so the cloud behaves as a set of gravitating point masses (no spurious impulses
# from the closely-seeded spawn positions).
sphere_material = chrono.ChContactMaterialNSC()
sphere_material.SetFriction(0.3)
sphere_material.SetRestitution(0.0)

# === Particle emitter === burst of spheres with Zhang-distributed diameters
emitter = chrono.ChParticleEmitter()
emitter.SetFlowControlMode(chrono.ChParticleEmitter.FLOW_PARTICLESPERSECOND)
emitter.SetParticlesPerSecond(PARTICLES_PER_SECOND)
emitter.SetUseParticleReservoir(True)
emitter.SetParticleReservoirAmount(TARGET_PARTICLES)  # hard cap on total spheres

# Shape creator: spheres with a Zhang diameter law and a constant density law.
sphere_creator = chrono.ChRandomShapeCreatorSpheres()
sphere_creator.SetDiameterDistribution(
    chrono.ChZhangDistribution(ZHANG_AVERAGE_DIAMETER, ZHANG_MINIMUM_DIAMETER))
sphere_creator.SetDensityDistribution(chrono.ChConstantDistribution(SPHERE_DENSITY))
sphere_creator.SetAddCollisionShape(ENABLE_CONTACT)
sphere_creator.SetAddVisualizationAsset(True)
emitter.SetParticleCreator(sphere_creator)

# Positioner: seed particles inside a cubic region centred on the origin.
positioner = chrono.ChRandomParticlePositionOnGeometry()
positioner.SetGeometry(chrono.ChBox(chrono.ChVector3d(SEED_BOX_SIZE, SEED_BOX_SIZE, SEED_BOX_SIZE)),
                       chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
emitter.SetParticlePositioner(positioner)

# No emitter velocity distribution: the spheres are released from rest and the
# only motion comes from the mutual gravitational attraction applied each step.

# Capture each created body so we can arm a force accumulator on it for the
# per-step gravity. The emitter's EmitParticles(sys, ...) already inserts every
# body into the system — the callback must NOT call sys.Add again (a second
# registration double-counts the body in the solver and makes it unstable).
particles = []  # holds every emitted sphere (and its accumulator index)


class _AddSphereCallback(chrono.ChRandomShapeCreator_AddBodyCallback):
    """Place each newly created sphere and arm its gravity-force accumulator."""

    def __init__(self, material):
        super().__init__()
        self._material = material

    def OnAddBody(self, body, acoord, creator):
        body.SetCoordsys(acoord)               # place the body at the sampled pose
        if ENABLE_CONTACT:
            body.GetCollisionModel().SetAllShapesMaterial(self._material)
        body.EnableCollision(ENABLE_CONTACT)   # OFF -> pure gravitating point mass
        vis_shape = body.GetVisualModel().GetShape(0)  # the auto-added sphere visual
        vis_shape.SetColor(chrono.ChColor(*SPHERE_COLOR))  # make the sphere clearly visible
        accum_idx = body.AddAccumulator()      # one accumulator for the gravity force
        particles.append((body, accum_idx))    # capture only; emitter adds to sys


add_callback = _AddSphereCallback(sphere_material)
sphere_creator.RegisterAddBodyCallback(add_callback)

# Emit the whole cloud in one burst at t=0. One emit call over a slice large
# enough (given PARTICLES_PER_SECOND) to fill the reservoir cap.
emitter.EmitParticles(sys, 1.0)

if not particles:
    raise RuntimeError("emitter produced no particles — check creator/positioner setup")

# Cache immutable per-body data once (mass never changes) — used every step.
particle_bodies = [p[0] for p in particles]   # cache: body handles, reused every step
particle_accum = [p[1] for p in particles]    # cache: accumulator indices, reused every step
particle_mass = [b.GetMass() for b in particle_bodies]  # cache: masses, constant for the run
n_particles = len(particle_bodies)


def compute_gravity_and_energy():
    """Apply pairwise Newtonian attraction to accumulators; return (KE, PE, COM).

    Softened inverse-square force on each ordered pair, plus the kinetic energy
    of every body and the (softened) pairwise gravitational potential energy.
    """
    # Reset accumulators before re-accumulating this step's pair forces.
    for body, idx in zip(particle_bodies, particle_accum):
        body.EmptyAccumulator(idx)

    positions = [b.GetPos() for b in particle_bodies]

    potential_energy = 0.0
    for i, j in itertools.combinations(range(n_particles), 2):
        d = positions[j] - positions[i]
        r2 = d.x * d.x + d.y * d.y + d.z * d.z + SOFTENING_SQ
        r = math.sqrt(r2)
        inv_r = 1.0 / r
        # F magnitude = G mA mB / r^2 ; direction from i toward j.
        f_mag = G_CONSTANT * particle_mass[i] * particle_mass[j] / r2
        fx = f_mag * d.x * inv_r
        fy = f_mag * d.y * inv_r
        fz = f_mag * d.z * inv_r
        force_on_i = chrono.ChVector3d(fx, fy, fz)
        particle_bodies[i].AccumulateForce(particle_accum[i], force_on_i, positions[i], False)
        particle_bodies[j].AccumulateForce(particle_accum[j], -force_on_i, positions[j], False)
        potential_energy += -G_CONSTANT * particle_mass[i] * particle_mass[j] * inv_r

    kinetic_energy = 0.0
    com = chrono.ChVector3d(0, 0, 0)
    total_mass = 0.0
    for body, mass in zip(particle_bodies, particle_mass):
        v = body.GetPosDt()
        kinetic_energy += 0.5 * mass * v.Length2()
        com += body.GetPos() * mass
        total_mass += mass
    com = com / total_mass
    return kinetic_energy, potential_energy, com


# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
if not HEADLESS:
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Gravitational particle cloud")
    vis.Initialize()                                    # Initialize FIRST (Irrlicht order)
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(*CAMERA_EYE), chrono.ChVector3d(*CAMERA_TARGET))
    vis.AddTypicalLights()
    vis.AddGrid(1.0, 1.0, 24, 24,
                chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -SEED_BOX_SIZE * 0.5), chrono.QUNIT),
                chrono.ChColor(0.4, 0.4, 0.4))          # ground reference grid below the cloud
    # The emitter-created spheres are not picked up by AttachSystem/BindAll alone
    # (they were added by EmitParticles, not through the visual model graph that
    # BindAll walks). Bind each one explicitly so the cloud actually renders.
    vis.BindAll()
    for _body in particle_bodies:
        vis.BindItem(_body)

# === Main loop === render-cadence outer loop; physics + energy in the inner batch
run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END  # short physics check when validating
os.makedirs("frames", exist_ok=True)  # guard against missing output dir

times = []
ke_series = []
pe_series = []
total_series = []

data_file = None
try:
    data_file = open("simulation_data.csv", "w", newline="")  # context-managed below via finally
except (OSError, IOError) as exc:  # disk full / permission denied
    raise RuntimeError(f"cannot open simulation_data.csv: {exc}")

try:
    writer = csv.writer(data_file)
    writer.writerow(["time", "kinetic_energy", "potential_energy", "total_energy",
                     "com_x", "com_y", "com_z", "num_particles"])

    frame = 0
    while (HEADLESS or vis.Run()) and sys.GetChTime() < run_end:
        if not HEADLESS:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index for ffmpeg
            frame += 1
        for _ in range(RENDER_EVERY):
            t = sys.GetChTime()
            kinetic_energy, potential_energy, com = compute_gravity_and_energy()
            total_energy = kinetic_energy + potential_energy
            # Per-step console trace of the energy budget (as requested).
            print(f"t={t:7.3f}  KE={kinetic_energy:12.4f}  "
                  f"PE={potential_energy:12.4f}  E={total_energy:12.4f}")
            writer.writerow([t, kinetic_energy, potential_energy, total_energy,
                             com.x, com.y, com.z, n_particles])
            times.append(t)
            ke_series.append(kinetic_energy)
            pe_series.append(potential_energy)
            total_series.append(total_energy)
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= run_end:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    if data_file is not None:
        data_file.close()  # flush partial CSV even if a step diverges

# === Post-processing === per-body snapshot + energy timeseries plot
with open("particles.csv", "w", newline="") as pf:
    pwriter = csv.writer(pf)
    pwriter.writerow(["id", "mass", "radius", "pos_x", "pos_y", "pos_z",
                      "vel_x", "vel_y", "vel_z", "speed"])
    for idx, (body, mass) in enumerate(zip(particle_bodies, particle_mass)):
        p = body.GetPos()
        v = body.GetPosDt()
        radius = (3.0 * mass / (4.0 * math.pi * SPHERE_DENSITY)) ** (1.0 / 3.0)
        pwriter.writerow([idx, mass, radius, p.x, p.y, p.z,
                          v.x, v.y, v.z, v.Length()])

if times:
    t_arr = np.array(times)
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(t_arr, np.array(ke_series), label="kinetic", color="tab:red")
    ax.plot(t_arr, np.array(pe_series), label="potential", color="tab:blue")
    ax.plot(t_arr, np.array(total_series), label="total", color="black", linestyle="--")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("energy [J]")
    ax.set_title(f"Gravitational cloud energy ({n_particles} particles)")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=120)
    plt.close(fig)

print(f"Done: {n_particles} particles, {len(times)} steps logged.")
