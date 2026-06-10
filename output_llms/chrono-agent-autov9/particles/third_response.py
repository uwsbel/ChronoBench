"""Three-body gravitational simulation (N-body problem) in PyChrono 9.0.x.

Models three spherical bodies that interact through mutual Newtonian
gravitational attraction (an inverse-square pairwise force), reproducing the
classic three-body problem. There is NO uniform downward gravity field: the only
forces are the pairwise attractions between the three spheres, so the trio drifts,
swings, and exchanges momentum purely through their own gravity.

System type: ChSystemNSC (rigid bodies, no contact collisions — the spheres are
point-like attractors and never touch at the chosen scale, so collision is off).

Bodies:
  - Sphere 1: starts at (10, 10, 0)   with velocity (0.5, 0, 0.1)
  - Sphere 2: starts at (-10, -10, 0)  with velocity (-0.5, 0, -0.1)
  - Sphere 3: starts at (0, 20, 0)     with velocity (0, -0.5, 0.2)

Each step the pairwise gravitational forces are recomputed and applied through a
per-body force accumulator; the total mechanical energy (kinetic + gravitational
potential) is logged and should stay approximately constant, which validates the
custom force integration.

Expected behavior: the three spheres follow curved, mutually-coupled trajectories
(no straight free-flight, no falling to a floor); positions/velocities are logged
to simulation_data.csv and particles.csv, and a time-series plot is written.
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

# === Constants & derived configuration ===
# Numeric literals for positions/velocities/masses live here only (no bare
# literals downstream). Time-stepping and render cadence are precomputed once.
TIME_STEP = 1.0e-3                 # physics step [s] (high-precision integration)
SIM_END = 20.0                     # simulation duration [s]
RENDER_FPS = 30.0                  # review-video frame rate [Hz]
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once: steps/frame

# Gravitational constant tuned for this scale (large masses, ~10 m separation)
# so the three bodies visibly curve/swing within SIM_END seconds.
G_CONST = 0.020                    # effective gravitational constant [m^3 kg^-1 s^-2]
SOFTENING = 1.5                    # Plummer softening length [m] (avoids 1/r^2 singularity)

SPHERE_RADIUS = 1.0                # visual sphere radius [m]
SPHERE_MASS = 1.0e3                # mass of each body [kg]
# Solid-sphere inertia I = (2/5) m r^2 (same about all axes); precomputed once.
SPHERE_INERTIA = 0.4 * SPHERE_MASS * SPHERE_RADIUS * SPHERE_RADIUS

# Final desired initial states for the three-body problem.
INIT_POS = [
    (10.0, 10.0, 0.0),    # Sphere 1
    (-10.0, -10.0, 0.0),  # Sphere 2
    (0.0, 20.0, 0.0),     # Sphere 3
]
INIT_VEL = [
    (0.5, 0.0, 0.1),      # Sphere 1
    (-0.5, 0.0, -0.1),    # Sphere 2
    (0.0, -0.5, 0.2),     # Sphere 3
]
BODY_COLORS = [
    chrono.ChColor(0.90, 0.25, 0.20),   # red
    chrono.ChColor(0.20, 0.45, 0.90),   # blue
    chrono.ChColor(0.20, 0.80, 0.35),   # green
]
NUM_BODIES = len(INIT_POS)

HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))  # fast, windowless validation run
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END  # short physics check when validating

# === System & gravity ===
# One ChSystemNSC. Uniform gravity is DISABLED — the only forces are the custom
# pairwise gravitational attractions applied via accumulators in the main loop.
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, 0.0))  # no global gravity field

# === Bodies ===
# Three spheres with explicit mass/inertia, initial position and velocity, and a
# single force accumulator each (added once here, reused every step).
bodies = []
accum_idx = []
for i in range(NUM_BODIES):
    body = chrono.ChBody()
    body.SetMass(SPHERE_MASS)
    body.SetInertiaXX(chrono.ChVector3d(SPHERE_INERTIA, SPHERE_INERTIA, SPHERE_INERTIA))
    body.SetPos(chrono.ChVector3d(*INIT_POS[i]))
    body.SetPosDt(chrono.ChVector3d(*INIT_VEL[i]))  # initial linear velocity
    body.EnableCollision(False)                     # point attractors: no contact

    sphere_vis = chrono.ChVisualShapeSphere(SPHERE_RADIUS)
    sphere_vis.SetColor(BODY_COLORS[i])
    body.AddVisualShape(sphere_vis)

    idx = body.AddAccumulator()   # one persistent force accumulator per body
    accum_idx.append(idx)
    sys.AddBody(body)
    bodies.append(body)


def compute_pairwise_gravity():
    """Apply mutual Newtonian attraction to every body's accumulator.

    Empties each accumulator, then for each unordered pair adds the softened
    inverse-square attraction F = G m_i m_j (r_j - r_i) / (|r|^2 + eps^2)^(3/2)
    to body i (and the equal-and-opposite force to body j). Returns the total
    gravitational potential energy of the configuration [J].
    """
    positions = [b.GetPos() for b in bodies]   # cache: one pose fetch per body per step
    for i in range(NUM_BODIES):
        bodies[i].EmptyAccumulator(accum_idx[i])  # clear last step's force

    potential = 0.0
    for i in range(NUM_BODIES):
        for j in range(i + 1, NUM_BODIES):
            d = positions[j] - positions[i]
            dist2 = d.x * d.x + d.y * d.y + d.z * d.z
            soft2 = dist2 + SOFTENING * SOFTENING
            inv_soft = 1.0 / math.sqrt(soft2)
            # force magnitude / distance, so multiplying by d gives the vector
            coeff = G_CONST * SPHERE_MASS * SPHERE_MASS * inv_soft / soft2
            fij = chrono.ChVector3d(d.x * coeff, d.y * coeff, d.z * coeff)
            # attraction on i points toward j (+d); on j points toward i (-d)
            bodies[i].AccumulateForce(accum_idx[i], fij,
                                      positions[i], False)   # world-frame force at COM
            bodies[j].AccumulateForce(accum_idx[j],
                                      chrono.ChVector3d(-fij.x, -fij.y, -fij.z),
                                      positions[j], False)
            potential += -G_CONST * SPHERE_MASS * SPHERE_MASS * inv_soft
    return potential


def total_kinetic():
    """Total translational kinetic energy of the three bodies [J]."""
    ke = 0.0
    for b in bodies:
        v = b.GetPosDt()
        ke += 0.5 * SPHERE_MASS * (v.x * v.x + v.y * v.y + v.z * v.z)
    return ke


# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
if not HEADLESS:
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Three-Body Gravitational Simulation")
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)   # Z-up world convention
    vis.Initialize()                                    # Initialize FIRST (Irrlicht order)
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()                                     # outdoor sky backdrop
    vis.AddCamera(chrono.ChVector3d(0, -38, 24),        # eye: close enough that motion reads clearly
                  chrono.ChVector3d(0, 0, 0))           # target: system barycenter region
    vis.AddTypicalLights()                              # standard lighting
    vis.AddGrid(2.0, 2.0, 40, 40,
                chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                chrono.ChColor(0.4, 0.4, 0.4))          # ground reference grid

# === Main loop ===
# Render-cadence outer loop: render/save one frame per RENDER_EVERY physics steps.
# Inside the inner batch we recompute pairwise gravity, log CSV, and step.
os.makedirs("frames", exist_ok=True)   # guard against missing output dir for review frames

times = []
ke_hist = []
pe_hist = []
te_hist = []
pos_hist = [[] for _ in range(NUM_BODIES)]  # each entry -> list of (x,y,z)

data_file = None
part_file = None
try:
    try:
        data_file = open("simulation_data.csv", "w", newline="")
        part_file = open("particles.csv", "w", newline="")
    except (OSError, IOError) as exc:   # disk full / permission denied on output dir
        print(f"Could not open CSV output: {exc}")
        raise

    data_writer = csv.writer(data_file)
    part_writer = csv.writer(part_file)

    data_header = ["time", "kinetic_energy", "potential_energy", "total_energy"]
    for i in range(NUM_BODIES):
        data_header += [f"x{i+1}", f"y{i+1}", f"z{i+1}",
                        f"vx{i+1}", f"vy{i+1}", f"vz{i+1}"]
    data_writer.writerow(data_header)
    part_writer.writerow(["time", "body_id", "x", "y", "z", "vx", "vy", "vz", "speed"])

    frame = 0
    while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
        if not HEADLESS:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index -> ffmpeg
            frame += 1

        for _ in range(RENDER_EVERY):
            t = sys.GetChTime()                 # cache: single time fetch reused this step
            pe = compute_pairwise_gravity()     # applies forces AND returns potential energy
            ke = total_kinetic()
            te = ke + pe

            row = [t, ke, pe, te]
            for i in range(NUM_BODIES):
                p = bodies[i].GetPos()
                v = bodies[i].GetPosDt()
                row += [p.x, p.y, p.z, v.x, v.y, v.z]
                speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
                part_writer.writerow([t, i + 1, p.x, p.y, p.z, v.x, v.y, v.z, speed])
                pos_hist[i].append((p.x, p.y, p.z))
            data_writer.writerow(row)

            times.append(t)
            ke_hist.append(ke)
            pe_hist.append(pe)
            te_hist.append(te)

            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= RUN_END:
                break

except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    # Flush + close CSV writers even if a step diverged mid-run.
    if data_file is not None:
        data_file.close()
    if part_file is not None:
        part_file.close()

# === Post-processing ===
# Energy-conservation check (top) + per-body trajectory projection (bottom),
# written to simulation_timeseries.png from the logged arrays.
if times:
    t_arr = np.array(times)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 9))

    ax1.plot(t_arr, np.array(ke_hist), label="kinetic")
    ax1.plot(t_arr, np.array(pe_hist), label="potential")
    ax1.plot(t_arr, np.array(te_hist), "k--", label="total")
    ax1.set(xlabel="time [s]", ylabel="energy [J]",
            title="Three-body energy (total should stay ~constant)")
    ax1.legend()
    ax1.grid(True)

    for i in range(NUM_BODIES):
        xs = [p[0] for p in pos_hist[i]]
        ys = [p[1] for p in pos_hist[i]]
        ax2.plot(xs, ys, label=f"sphere {i+1}")
        ax2.plot(xs[0], ys[0], "o")
    ax2.set(xlabel="x [m]", ylabel="y [m]", title="Trajectories (XY projection)")
    ax2.axis("equal")
    ax2.legend()
    ax2.grid(True)

    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=110)
    plt.close(fig)

print(f"Done: {len(times)} steps logged, final t={times[-1] if times else 0.0:.3f} s")
