"""Single mass-spring-damper system in PyChrono 9.0.x (Irrlicht renderer).

Model
-----
A single rigid mass (a box) is suspended from a fixed ground anchor by a linear
spring-damper modeled with ``chrono.ChLinkTSDA``. The translational spring-damper
applies a force along its axis according to F = -k*(len - rest_len) - c*len_dot,
using default linear spring/damper coefficients. Gravity acts along -Z, so the
mass is pulled down, stretches the spring, and settles into a damped oscillation
about its static-equilibrium height before coming to rest.

System type
-----------
``ChSystemNSC`` (non-smooth contact). There is no contact in this scene — the
only coupling between the mass and ground is the TSDA link — so no collision
system is configured. Spring-damper chains are stiff, so the PSOR iterative
solver is used with warm-starting enabled for stable convergence.

Main bodies
-----------
- ``ground``  : a fixed anchor body (small box) at the top, the upper spring end.
- ``mass``    : the dynamic box hanging below, the lower spring end / protagonist.

Expected behavior
------------------
The mass drops from its initial position, the spring stretches, and the system
performs a decaying vertical (Z) oscillation that converges to a static
equilibrium where spring force balances gravity. The Z position and velocity
logged to CSV show the characteristic damped sinusoid.
"""

# === Imports ===
import os
import csv
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

import matplotlib
matplotlib.use("Agg")  # headless plotting backend (no display needed for the PNG)
import matplotlib.pyplot as plt


# === Named constants (geometry, physics, timing) ===
# Body geometry / mass
MASS_BOX_SIZE = 0.4          # full edge length of the dynamic mass cube [m]
MASS_VALUE = 1.0             # mass of the dynamic body [kg]
ANCHOR_BOX_SIZE = 0.2        # full edge length of the fixed anchor cube [m]

# Spring (default linear spring-damper parameters per the task)
SPRING_REST_LENGTH = 1.0     # natural (unstretched) spring length [m]
SPRING_COEFFICIENT = 50.0    # linear stiffness k [N/m] (default linear value)
DAMPING_COEFFICIENT = 1.0    # linear damping c [N*s/m] (default linear value)
SPRING_COIL_RADIUS = 0.08    # visual coil radius of the rendered spring [m]
SPRING_RESOLUTION = 80       # visual coil mesh resolution
SPRING_TURNS = 15            # visual number of coils

# World / gravity
GRAVITY_Z = -9.81            # gravitational acceleration along Z [m/s^2]

# Anchor and initial mass positions (derived geometry, computed once below)
ANCHOR_Z = 2.0                                   # ground anchor height [m]
INITIAL_GAP = SPRING_REST_LENGTH                 # start at rest length (spring relaxed)
MASS_INITIAL_Z = ANCHOR_Z - INITIAL_GAP          # initial mass height [m]

# Solver / timing
SOLVER_MAX_ITERATIONS = 100  # PSOR iteration cap per step
SOLVER_TOLERANCE = 1e-10     # PSOR convergence tolerance
TIME_STEP = 1e-3             # integration step [s]
SIM_END = 10.0               # total simulated time [s]
RENDER_FPS = 50.0            # review-frame cadence [frames/s]

# Inertia for a solid box: I = (1/12) * m * (a^2 + b^2) about each axis (cube here)
MASS_INERTIA = (1.0 / 12.0) * MASS_VALUE * (MASS_BOX_SIZE ** 2 + MASS_BOX_SIZE ** 2)

# Fast, windowless validation run (set SIMBENCH_VALIDATE=1): skips the Irrlicht
# window and runs a short bounded physics check that still writes the CSVs.
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))

# Precomputed once: physics steps between rendered frames, and the run end time.
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END         # short check when validating


def main():
    # === System & gravity === one NSC system; gravity pulls the mass down -Z.
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, GRAVITY_Z))

    # === Solver configuration === PSOR + warm start: stiff spring-damper stability.
    sys.SetSolverType(chrono.ChSolver.Type_PSOR)
    solver = sys.GetSolver().AsIterative()
    solver.SetMaxIterations(SOLVER_MAX_ITERATIONS)
    solver.SetTolerance(SOLVER_TOLERANCE)
    solver.EnableWarmStart(True)  # reuse previous-step solution -> spring convergence

    # === Bodies === fixed ground anchor (top) and the dynamic suspended mass (bottom).
    ground = chrono.ChBody()
    ground.SetFixed(True)
    ground.SetPos(chrono.ChVector3d(0, 0, ANCHOR_Z))
    ground.EnableCollision(False)
    anchor_shape = chrono.ChVisualShapeBox(ANCHOR_BOX_SIZE, ANCHOR_BOX_SIZE, ANCHOR_BOX_SIZE)
    anchor_shape.SetColor(chrono.ChColor(0.2, 0.2, 0.2))
    ground.AddVisualShape(anchor_shape)
    sys.AddBody(ground)

    mass = chrono.ChBody()
    mass.SetMass(MASS_VALUE)
    mass.SetInertiaXX(chrono.ChVector3d(MASS_INERTIA, MASS_INERTIA, MASS_INERTIA))
    mass.SetPos(chrono.ChVector3d(0, 0, MASS_INITIAL_Z))
    mass.EnableCollision(False)
    mass_shape = chrono.ChVisualShapeBox(MASS_BOX_SIZE, MASS_BOX_SIZE, MASS_BOX_SIZE)
    mass_shape.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
    mass.AddVisualShape(mass_shape)
    sys.AddBody(mass)

    # === Spring-damper link (ChLinkTSDA) === the only coupling between mass and ground.
    # Endpoints given in each body's LOCAL frame (rel_frames=True): both at body origin.
    spring = chrono.ChLinkTSDA()
    spring.Initialize(mass, ground, True,
                      chrono.ChVector3d(0, 0, 0),
                      chrono.ChVector3d(0, 0, 0))
    spring.SetRestLength(SPRING_REST_LENGTH)
    spring.SetSpringCoefficient(SPRING_COEFFICIENT)
    spring.SetDampingCoefficient(DAMPING_COEFFICIENT)
    sys.AddLink(spring)
    # Visual shape MUST be added to the LINK itself, not to a body.
    spring.AddVisualShape(
        chrono.ChVisualShapeSpring(SPRING_COIL_RADIUS, SPRING_RESOLUTION, SPRING_TURNS)
    )

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid.
    # Built only for an on-screen run; the headless validation gate skips the window.
    vis = None
    if not HEADLESS:
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(sys)
        vis.SetCameraVertical(chrono.CameraVerticalDir_Z)   # gravity along -Z => Z-up
        vis.SetWindowSize(1280, 720)
        vis.SetWindowTitle("Mass-Spring-Damper (ChLinkTSDA)")
        vis.Initialize()                                    # Initialize FIRST (Irrlicht)
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        vis.AddSkyBox()                                     # standard outdoor sky backdrop
        vis.AddCamera(chrono.ChVector3d(3, -4, 1.5),
                      chrono.ChVector3d(0, 0, 1.0))         # eye, look-at (AFTER Initialize)
        vis.AddTypicalLights()                              # standard lighting
        vis.AddGrid(0.5, 0.5, 40, 40,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))          # ground reference grid

    # === Output directories === guard against a missing output dir before the loop.
    os.makedirs("frames", exist_ok=True)  # review PNG frames -> ffmpeg mp4 later
    os.makedirs("cam", exist_ok=True)     # motion log for the moving mass

    # cache: fetched once, reused every step (avoids re-resolving the getter in the loop)
    get_time = sys.GetChTime  # cache: bound method, reused every physics step

    # Sanity check: the mass must start below the anchor (spring hangs downward).
    assert MASS_INITIAL_Z < ANCHOR_Z, "mass must start below the fixed anchor"

    # === Main loop === render-cadence outer loop; physics + CSV logging in inner batch.
    sim_csv = None
    motion_csv = None
    times, z_pos, z_vel, spring_len = [], [], [], []
    try:
        sim_csv = open("simulation_data.csv", "w", newline="")          # main physics log
        motion_csv = open(os.path.join("cam", "motion_log.csv"), "w", newline="")  # mover pose
        sim_writer = csv.writer(sim_csv)
        motion_writer = csv.writer(motion_csv)
        sim_writer.writerow(["time", "mass_z", "mass_vz", "spring_length"])
        motion_writer.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz"])

        frame = 0
        while (HEADLESS or vis.Run()) and get_time() < RUN_END:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index
                frame += 1
            for _ in range(RENDER_EVERY):
                t = get_time()
                pos = mass.GetPos()
                vel = mass.GetPosDt()
                length = spring.GetLength()
                sim_writer.writerow([t, pos.z, vel.z, length])
                motion_writer.writerow([t, "mass", pos.x, pos.y, pos.z, vel.x, vel.y, vel.z])
                times.append(t)
                z_pos.append(pos.z)
                z_vel.append(vel.z)
                spring_len.append(length)
                sys.DoStepDynamics(TIME_STEP)
                if get_time() >= RUN_END:
                    break
    except (OSError, IOError) as exc:           # disk / permission error opening or writing CSV
        import traceback
        traceback.print_exc()
        raise
    except (RuntimeError, ValueError) as exc:    # solver divergence / invalid simulation state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # Flush + close any open writers even if a step diverged mid-run.
        if sim_csv is not None:
            sim_csv.close()
        if motion_csv is not None:
            motion_csv.close()

    # === Post-processing === plot the logged time series to a PNG for review.
    if times:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
        ax1.plot(times, z_pos, label="mass Z [m]", color="tab:red")
        ax1.plot(times, spring_len, label="spring length [m]", color="tab:blue")
        ax1.set_ylabel("position / length [m]")
        ax1.legend(loc="best")
        ax1.grid(True)
        ax2.plot(times, z_vel, label="mass Vz [m/s]", color="tab:green")
        ax2.set_xlabel("time [s]")
        ax2.set_ylabel("velocity [m/s]")
        ax2.legend(loc="best")
        ax2.grid(True)
        fig.suptitle("Mass-Spring-Damper (ChLinkTSDA) response")
        fig.tight_layout()
        with open("simulation_timeseries.png", "wb") as png_file:
            fig.savefig(png_file, format="png", dpi=110)
        plt.close(fig)


if __name__ == "__main__":
    main()
