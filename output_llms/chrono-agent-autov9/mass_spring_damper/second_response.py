"""Mass-spring-damper simulation with two parallel oscillators (PyChrono 9.0.1, Irrlicht).

Models two identical point masses, each suspended from a fixed ground anchor by a
translational spring-damper (ChLinkTSDA), in a ChSystemNSC with gravity along -Y:

  * body_1 / spring_1: a classic spring-damper that uses the link's built-in
    SetSpringCoefficient / SetDampingCoefficient (direct-coefficient method).
  * body_2 / spring_2: an equivalent spring-damper whose force is supplied by a
    custom force functor (MySpringForce, a chrono.ForceFunctor subclass) using
    spring_coef = 50 and damping_coef = 1.

Two visual reference spheres (sph_1, sph_2) mark the ground anchor points. Both
masses start displaced below their rest length and oscillate vertically, decaying
toward static equilibrium; the two methods should produce matching motion.

System type: NSC (no contact — pure spring/joint mechanics, collision system omitted).
Expected behavior: damped vertical oscillation of both masses about gravity-shifted
equilibria, with body_1 and body_2 tracking each other closely.
"""

import os
import csv
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / physics / timing (no bare literals downstream)
TIME_STEP = 1.0e-3          # integration step [s]
SIM_END = 4.0               # total simulated time [s]
RENDER_FPS = 50.0           # review-frame cadence [frames/s]

SPRING_COEF = 50.0          # spring stiffness k [N/m] (custom functor + direct)
DAMPING_COEF = 1.0          # damping coefficient c [N*s/m]
REST_LENGTH = 1.5           # spring free length [m]

MASS = 1.0                  # point-mass value [kg]
BODY_RADIUS = 0.1           # visual sphere radius of each mass [m]
GROUND_SPH_RADIUS = 0.05    # visual sphere radius of ground anchor markers [m]

# Ground anchor (spring top) points; sph_1 mirrors to sph_2 one unit along +X.
ANCHOR_1 = chrono.ChVector3d(0.0, 0.0, 0.0)
ANCHOR_2 = chrono.ChVector3d(1.0, 0.0, 0.0)

# Masses start hanging one REST_LENGTH below their anchors (precomputed once).
BODY_1_POS = chrono.ChVector3d(ANCHOR_1.x, ANCHOR_1.y - REST_LENGTH, ANCHOR_1.z)
BODY_2_POS = chrono.ChVector3d(ANCHOR_2.x, ANCHOR_2.y - REST_LENGTH, ANCHOR_2.z)

SPRING_COIL_RADIUS = 0.08   # visual coil radius
SPRING_RESOLUTION = 80      # coil polyline resolution
SPRING_TURNS = 15           # number of coil turns

# Headless validation gate: fast, windowless physics check when SIMBENCH_VALIDATE set.
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))


# === Custom force functor === evaluates spring_2 force from custom coefficients
class MySpringForce(chrono.ForceFunctor):
    """Linear spring-damper force used by spring_2.

    Returns the scalar actuator force along the TSDA axis:
        F = -k * (length - rest_length) - c * vel
    (positive force pushes the endpoints apart, restoring toward rest_length).
    """

    def __init__(self, spring_coef, damping_coef):
        super().__init__()
        self.spring_coef = spring_coef      # cache: stiffness stored once on the functor
        self.damping_coef = damping_coef    # cache: damping stored once on the functor

    def evaluate(self, time, rest_length, length, vel, link):
        return -self.spring_coef * (length - rest_length) - self.damping_coef * vel


def make_mass_body(pos):
    """Create one point-mass rigid body with a visual sphere at the given position."""
    body = chrono.ChBody()
    body.SetMass(MASS)
    # Solid-sphere inertia: (2/5) m r^2 about each axis.
    inertia = (2.0 / 5.0) * MASS * BODY_RADIUS * BODY_RADIUS
    body.SetInertiaXX(chrono.ChVector3d(inertia, inertia, inertia))
    body.SetPos(pos)
    sphere = chrono.ChVisualShapeSphere(BODY_RADIUS)
    sphere.SetColor(chrono.ChColor(0.2, 0.4, 0.9))
    body.AddVisualShape(sphere)
    return body


def make_spring(body, anchor_world):
    """Create a TSDA between `body` (local origin) and ground (`anchor_world`)."""
    spring = chrono.ChLinkTSDA()
    # 5-arg world form (local=False): loc1 = body's world origin, loc2 = ground anchor.
    spring.Initialize(body, ground, False,
                      body.GetPos(), anchor_world)
    spring.SetRestLength(REST_LENGTH)
    spring.AddVisualShape(
        chrono.ChVisualShapeSpring(SPRING_COIL_RADIUS, SPRING_RESOLUTION, SPRING_TURNS))
    return spring


# === System & gravity === one NSC system, gravity along -Y (no contact -> no collision system)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Spring-damper chains are stiff: PSOR + warm start for stable convergence.
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
_solver = sys.GetSolver().AsIterative()   # cache: iterative-solver handle fetched once
_solver.SetMaxIterations(100)
_solver.SetTolerance(1e-10)
_solver.EnableWarmStart(True)             # critical for spring convergence

# === Bodies === fixed ground (with two anchor marker spheres) + two oscillating masses
ground = chrono.ChBody()
ground.SetFixed(True)
sph_1 = chrono.ChVisualShapeSphere(GROUND_SPH_RADIUS)
sph_1.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
ground.AddVisualShape(sph_1, chrono.ChFramed(ANCHOR_1, chrono.QUNIT))   # sph_1 at (0,0,0)
sph_2 = chrono.ChVisualShapeSphere(GROUND_SPH_RADIUS)
sph_2.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
ground.AddVisualShape(sph_2, chrono.ChFramed(ANCHOR_2, chrono.QUNIT))   # sph_2 at (1,0,0)
sys.AddBody(ground)

body_1 = make_mass_body(BODY_1_POS)
sys.AddBody(body_1)
body_2 = make_mass_body(BODY_2_POS)
sys.AddBody(body_2)

# === Joints / constraints === two TSDAs: direct-coefficient vs custom force functor
# spring_1: built-in spring/damping coefficients (direct method).
spring_1 = make_spring(body_1, ANCHOR_1)
spring_1.SetSpringCoefficient(SPRING_COEF)
spring_1.SetDampingCoefficient(DAMPING_COEF)
sys.AddLink(spring_1)

# spring_2: force supplied by the custom MySpringForce functor.
spring_2 = make_spring(body_2, ANCHOR_2)
my_force = MySpringForce(SPRING_COEF, DAMPING_COEF)
spring_2.RegisterForceFunctor(my_force)
sys.AddLink(spring_2)

# === Derived loop constants === precomputed once (never recomputed in the loop)
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # physics steps per frame
run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END           # short check when validating

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
if not HEADLESS:
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Mass-Spring-Damper: two parallel oscillators")
    vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # gravity is along -Y
    vis.Initialize()                                    # Initialize FIRST (Irrlicht order)
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.5, -1.0, 4.0), chrono.ChVector3d(0.5, -1.0, 0.0))
    vis.AddTypicalLights()
    vis.AddGrid(0.5, 0.5, 40, 40,
                chrono.ChCoordsysd(chrono.ChVector3d(0.5, 0.0, 0.0), chrono.QUNIT),
                chrono.ChColor(0.4, 0.4, 0.4))          # ground reference grid

# === Main loop === render-cadence outer loop; physics + CSV logging in the inner batch
os.makedirs("frames", exist_ok=True)   # guard against missing output dir
os.makedirs("cam", exist_ok=True)

data_file = None
motion_file = None
times, y1_hist, y2_hist, len1_hist, len2_hist = [], [], [], [], []
try:
    try:
        data_file = open("simulation_data.csv", "w", newline="")
        motion_file = open("cam/motion_log.csv", "w", newline="")
    except (OSError, IOError) as exc:   # disk full / permission denied
        print("Failed to open output CSV:", exc)
        raise

    data_writer = csv.writer(data_file)
    data_writer.writerow(["time", "body1_y", "body2_y",
                          "spring1_length", "spring2_length",
                          "spring1_force", "spring2_force"])
    motion_writer = csv.writer(motion_file)
    motion_writer.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz"])

    frame = 0
    while (HEADLESS or vis.Run()) and sys.GetChTime() < run_end:
        if not HEADLESS:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive -> ffmpeg
            frame += 1

        for _ in range(render_every):
            t = sys.GetChTime()
            p1 = body_1.GetPos()      # cache: pose fetched once per step, reused for logs
            p2 = body_2.GetPos()      # cache: pose fetched once per step, reused for logs
            v1 = body_1.GetPosDt()
            v2 = body_2.GetPosDt()
            data_writer.writerow([t, p1.y, p2.y,
                                  spring_1.GetLength(), spring_2.GetLength(),
                                  spring_1.GetForce(), spring_2.GetForce()])
            motion_writer.writerow([t, "body_1", p1.x, p1.y, p1.z, v1.x, v1.y, v1.z])
            motion_writer.writerow([t, "body_2", p2.x, p2.y, p2.z, v2.x, v2.y, v2.z])
            times.append(t)
            y1_hist.append(p1.y)
            y2_hist.append(p2.y)
            len1_hist.append(spring_1.GetLength())
            len2_hist.append(spring_2.GetLength())

            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= run_end:
                break

except (RuntimeError, ValueError) as exc:   # solver divergence / bad numeric state
    import traceback
    traceback.print_exc()
    raise
finally:
    # Flush + close any open writers even if a step diverged mid-run.
    if data_file is not None:
        data_file.close()
    if motion_file is not None:
        motion_file.close()

# === Post-processing === plot the logged time series to a PNG
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
ax1.plot(times, y1_hist, label="body_1 y (direct coeff)")
ax1.plot(times, y2_hist, "--", label="body_2 y (custom functor)")
ax1.set_ylabel("vertical position [m]")
ax1.legend()
ax1.grid(True)
ax2.plot(times, len1_hist, label="spring_1 length")
ax2.plot(times, len2_hist, "--", label="spring_2 length")
ax2.axhline(REST_LENGTH, color="gray", lw=0.8, label="rest length")
ax2.set_xlabel("time [s]")
ax2.set_ylabel("spring length [m]")
ax2.legend()
ax2.grid(True)
fig.suptitle("Mass-spring-damper oscillation: direct vs custom-functor force")
fig.tight_layout()
fig.savefig("simulation_timeseries.png", dpi=110)

print("Done. steps logged:", len(times),
      "final body_1 y:", y1_hist[-1] if y1_hist else float("nan"))
