"""Three-mass spring-damper chain (PyChrono 9.0.x, Irrlicht).

Models a horizontal chain of three rigid masses (body_1, body_2, body_3)
connected in series by linear spring-dampers (ChLinkTSDA). A fixed wall anchors
the chain: wall -> body_1 -> body_2 -> body_3. Each spring carries a coil
visual (ChVisualShapeSpring). The chain is displaced from equilibrium at the
start (the masses are spaced wider than the spring rest length), so the system
oscillates and the damping coefficients bleed energy until the masses settle
toward their static equilibrium spacing.

System type: ChSystemNSC (non-smooth contact), but the mechanism is pure
spring-damper topology with prismatic guides constraining motion to the X axis;
no contact occurs. Gravity is along -Z; the masses ride at a constant height so
gravity does not drive the longitudinal (X) dynamics. The PSOR iterative solver
with warm start keeps the stiff spring chain stable.

Expected behavior: the three masses oscillate longitudinally along X about their
equilibrium positions, coupled through the springs, and the oscillation decays
over time due to the spring-damper damping.
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

# === Named constants: timing, geometry, physics ===
TIME_STEP = 1.0e-3                 # integration step [s]
SIM_END = 8.0                      # total simulated time [s]
RENDER_FPS = 30.0                  # review-frame cadence [frames/s]

BODY_MASS = 1.0                    # mass of each moving body [kg]
BODY_HALF = 0.25                   # half-size (cube half-extent) of each body [m]
BODY_FULL = 2.0 * BODY_HALF        # full box extent of each body [m]
BODY_DENSITY = BODY_MASS / (BODY_FULL ** 3)  # density so cube has BODY_MASS [kg/m^3]

SPRING_K = 80.0                    # spring stiffness for each link [N/m]
SPRING_C = 4.0                     # damping coefficient for each link [N*s/m]
REST_LENGTH = 1.0                  # natural (unstretched) length of each spring [m]

WALL_X = 0.0                       # X position of the fixed anchor wall [m]
CHAIN_Z = 0.0                      # constant ride height of the chain (gravity is -Z) [m]
# Initial spacing wider than the rest length -> springs start stretched -> motion.
INIT_SPACING = 1.4                 # initial center-to-center gap between masses [m]

# Derived initial positions (computed once; no bare literals downstream).
BODY1_X = WALL_X + INIT_SPACING
BODY2_X = BODY1_X + INIT_SPACING
BODY3_X = BODY2_X + INIT_SPACING

# Spring coil visual parameters.
COIL_RADIUS = 0.08                 # coil radius of the rendered spring [m]
COIL_RESOLUTION = 80               # coil mesh resolution
COIL_TURNS = 12                    # number of coil turns

# Box inertia for a solid cube of side BODY_FULL: I = (1/6) m a^2 about each axis.
BODY_INERTIA = (1.0 / 6.0) * BODY_MASS * (BODY_FULL ** 2)

# Render cadence (precomputed once; never recomputed in the loop).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # steps per frame

HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))  # fast windowless validation run


def make_mass(system, name, pos_x):
    """Create one dynamic cube mass on the chain at world X = pos_x."""
    body = chrono.ChBody()
    body.SetName(name)
    body.SetMass(BODY_MASS)
    body.SetInertiaXX(chrono.ChVector3d(BODY_INERTIA, BODY_INERTIA, BODY_INERTIA))
    body.SetPos(chrono.ChVector3d(pos_x, 0.0, CHAIN_Z))
    body.EnableCollision(False)  # pure spring-damper topology; no contact
    box = chrono.ChVisualShapeBox(BODY_FULL, BODY_FULL, BODY_FULL)
    box.SetColor(chrono.ChColor(0.2, 0.5, 0.9))
    body.AddVisualShape(box)
    system.AddBody(body)
    return body


def make_spring(system, body_a, body_b):
    """Connect two bodies (centers) with a visualized linear spring-damper."""
    spring = chrono.ChLinkTSDA()
    # Connect body centers (local origin on each body); rel-frame flag True.
    spring.Initialize(body_a, body_b, True,
                      chrono.ChVector3d(0, 0, 0),
                      chrono.ChVector3d(0, 0, 0))
    spring.SetRestLength(REST_LENGTH)
    spring.SetSpringCoefficient(SPRING_K)
    spring.SetDampingCoefficient(SPRING_C)
    system.AddLink(spring)
    # Visual coil must be added to the LINK itself, not a body.
    spring.AddVisualShape(chrono.ChVisualShapeSpring(COIL_RADIUS, COIL_RESOLUTION, COIL_TURNS))
    return spring


def add_x_guide(system, ground, body):
    """Constrain a body to slide only along world X (prismatic to ground)."""
    # Prismatic uses frame local +Z as the sliding axis -> map +Z onto world +X.
    prismatic = chrono.ChLinkLockPrismatic()
    frame = chrono.ChFramed(body.GetPos(), chrono.Q_ROTATE_Z_TO_X)
    prismatic.Initialize(body, ground, frame)
    system.AddLink(prismatic)
    return prismatic


def main():
    # === System & gravity === single NSC system; gravity along -Z (chain rides level) ===
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    # PSOR + warm start: required for a stiff spring-damper chain to stay stable.
    system.SetSolverType(chrono.ChSolver.Type_PSOR)
    solver = system.GetSolver().AsIterative()
    solver.SetMaxIterations(100)
    solver.SetTolerance(1e-10)
    solver.EnableWarmStart(True)  # reuse previous step solution -> spring convergence

    # === Bodies === fixed anchor wall + three series masses ===
    wall = chrono.ChBody()
    wall.SetName("wall")
    wall.SetFixed(True)
    wall.SetPos(chrono.ChVector3d(WALL_X, 0.0, CHAIN_Z))
    wall_vis = chrono.ChVisualShapeBox(0.2, 1.0, 1.0)
    wall_vis.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
    wall.AddVisualShape(wall_vis)
    system.AddBody(wall)

    # Ground reference body for the prismatic X-guides (fixed, invisible frame).
    ground = chrono.ChBody()
    ground.SetName("ground")
    ground.SetFixed(True)
    system.AddBody(ground)

    body_1 = make_mass(system, "body_1", BODY1_X)
    body_2 = make_mass(system, "body_2", BODY2_X)
    body_3 = make_mass(system, "body_3", BODY3_X)

    # === Joints / constraints === guide each mass to pure X translation ===
    add_x_guide(system, ground, body_1)
    add_x_guide(system, ground, body_2)
    add_x_guide(system, ground, body_3)

    # === Springs === series spring-dampers wall->1, 1->2, 2->3 ===
    make_spring(system, wall, body_1)
    make_spring(system, body_1, body_2)
    make_spring(system, body_2, body_3)

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid ===
    vis = None
    if not HEADLESS:
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(system)
        vis.SetWindowSize(1280, 720)
        vis.SetWindowTitle("Three-mass spring-damper chain")
        vis.SetCameraVertical(chrono.CameraVerticalDir_Z)  # gravity along -Z
        vis.Initialize()                                   # Initialize FIRST (Irrlicht)
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddCamera(chrono.ChVector3d(2.0, -6.0, 3.0),
                      chrono.ChVector3d(2.5, 0.0, 0.0))     # AFTER Initialize
        vis.AddTypicalLights()
        vis.AddGrid(0.5, 0.5, 40, 40,
                    chrono.ChCoordsysd(chrono.ChVector3d(2.5, 0, -BODY_HALF), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))           # ground reference grid

    # Cache body handles reused every step (avoid repeated lookups in the loop).
    tracked = [body_1, body_2, body_3]  # cache: fetched once, reused every step

    run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END  # short physics check when validating

    os.makedirs("frames", exist_ok=True)  # guard against missing output dir
    os.makedirs("cam", exist_ok=True)     # motion_log lives under cam/

    data_file = None
    motion_file = None
    times = []
    pos_hist = [[], [], []]
    try:
        # Open both CSVs up-front so they always flush/close in finally.
        try:
            data_file = open("simulation_data.csv", "w", newline="")
            motion_file = open("cam/motion_log.csv", "w", newline="")
        except (OSError, IOError) as exc:  # disk full / permission denied
            print(f"Could not open output CSV: {exc}")
            raise

        data_writer = csv.writer(data_file)
        data_writer.writerow(["time", "x1", "x2", "x3", "vx1", "vx2", "vx3"])
        motion_writer = csv.writer(motion_file)
        motion_writer.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz"])

        # === Main loop === render-cadence outer loop; physics batch between frames ===
        frame = 0
        while (HEADLESS or vis.Run()) and system.GetChTime() < run_end:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index
                frame += 1
            for _ in range(RENDER_EVERY):
                t = system.GetChTime()
                xs = [b.GetPos().x for b in tracked]
                vxs = [b.GetPosDt().x for b in tracked]
                data_writer.writerow([t, xs[0], xs[1], xs[2], vxs[0], vxs[1], vxs[2]])
                for b in tracked:
                    p = b.GetPos()
                    v = b.GetPosDt()
                    motion_writer.writerow([t, b.GetName(), p.x, p.y, p.z, v.x, v.y, v.z])
                times.append(t)
                for i in range(3):
                    pos_hist[i].append(xs[i])
                system.DoStepDynamics(TIME_STEP)
                if system.GetChTime() >= run_end:
                    break
    except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # Flush partial CSVs even if a step diverged mid-run.
        if data_file is not None:
            data_file.close()
        if motion_file is not None:
            motion_file.close()

    # === Post-processing === plot mass X-positions vs time ===
    if times:
        arr_t = np.array(times)
        plt.figure(figsize=(10, 6))
        for i in range(3):
            plt.plot(arr_t, np.array(pos_hist[i]), label=f"body_{i+1} x")
        plt.xlabel("time [s]")
        plt.ylabel("X position [m]")
        plt.title("Three-mass spring-damper chain: longitudinal positions")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.savefig("simulation_timeseries.png", dpi=120)
        plt.close()
        print(f"Wrote {len(times)} samples; final X = "
              f"{pos_hist[0][-1]:.4f}, {pos_hist[1][-1]:.4f}, {pos_hist[2][-1]:.4f}")


if __name__ == "__main__":
    main()
