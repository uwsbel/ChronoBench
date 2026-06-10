"""ANCF flexible cable hanging from a fixed truss, loaded by a tip force.

Model
-----
A single flexible cable built from ANCF (Absolute Nodal Coordinate Formulation)
gradient-deficient beam elements (``fea.ChBuilderCableANCF``). The cable is laid
out horizontally and its rear node is pinned to a fixed rigid truss via a
``fea.ChLinkNodeFrame`` constraint, so the cable behaves as a cantilever. A
constant downward force is applied to the free front node, and the cable sags
and oscillates under gravity plus that load until it reaches a damped static
deflection set by its Rayleigh damping.

System / solver
---------------
* ``chrono.ChSystemSMC`` (smooth contact; FEA stiffness needs the SMC family).
* Iterative MINRES solver (``chrono.ChSolverMINRES``) with a diagonal
  preconditioner and warm start, as required for this run.
* HHT implicit timestepper for stable integration of the stiff beam elements.

Cable section
-------------
Circular ANCF cable section with Rayleigh damping ``0.0001`` (light structural
damping so the oscillation decays smoothly to a static sag).

Expected behavior
-----------------
The pinned rear node stays fixed; the free front node drops under the applied
``(0, -0.7, 0)`` N force plus gravity, swings down, and settles into a steady
sagging cantilever shape. Front-node Y position is logged to CSV and plotted.
"""

import os
import csv
import math

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / physics / solver parameters (no bare literals downstream)
GRAVITY = chrono.ChVector3d(0, -9.81, 0)        # gravity along -Y (cable hangs in Y)
CABLE_LENGTH = 1.0                               # m, total cable span
N_ELEMENTS = 10                                  # ANCF beam elements along the cable
CABLE_DIAMETER = 0.015                           # m, circular cross-section diameter
CABLE_DENSITY = 1000.0                           # kg/m^3
CABLE_YOUNG_MODULUS = 0.01e9                     # Pa, soft cable so deflection is visible
RAYLEIGH_DAMPING = 0.0001                        # structural damping for the cable section
TIP_FORCE = chrono.ChVector3d(0, -0.7, 0)        # N, downward load on the free front node

CABLE_START = chrono.ChVector3d(0, 0, 0)         # pinned (rear) end
CABLE_END = chrono.ChVector3d(CABLE_LENGTH, 0, 0)  # free (front) end before loading

TIME_STEP = 1e-3                                 # s, integration step
SIM_END = 5.0                                    # s, simulated duration
RENDER_FPS = 50.0                                # frames per second for the review video

SOLVER_MAX_ITERS = 200                           # MINRES iteration cap
SOLVER_TOLERANCE = 1e-10                         # MINRES convergence tolerance

# precomputed once: physics steps between rendered frames (avoid per-step vis.Run())
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))

# Fast, windowless validation run (short bounded sim, no Irrlicht window).
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))

# Strong references kept alive against SWIG garbage collection of FEA objects.
KEEPALIVE = []


def main():
    # === System & gravity === SMC system required by the FEA stiffness/contact family
    sys = chrono.ChSystemSMC()
    sys.SetGravitationalAcceleration(GRAVITY)

    # === Solver === iterative MINRES (per this run's configuration)
    solver = chrono.ChSolverMINRES()
    if solver.GetType() == chrono.ChSolver.Type_MINRES:
        print("Using MINRES solver")
    solver.SetMaxIterations(SOLVER_MAX_ITERS)        # iteration cap
    solver.SetTolerance(SOLVER_TOLERANCE)            # convergence tolerance
    solver.EnableDiagonalPreconditioner(True)        # speeds up MINRES on FEA systems
    solver.EnableWarmStart(True)                     # reuse last solution as initial guess
    solver.SetVerbose(False)                         # quiet solver output
    sys.SetSolver(solver)

    # === Timestepper === HHT implicit integration for the stiff ANCF beam elements
    sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)   # set HHT by type (this build)

    # === Fixed truss === rigid anchor the cable's rear node is pinned to
    truss = chrono.ChBody()
    truss.SetFixed(True)
    truss.SetPos(CABLE_START)
    sys.Add(truss)

    # === Cable section === circular ANCF cable with light Rayleigh damping
    section = fea.ChBeamSectionCable()
    section.SetDiameter(CABLE_DIAMETER)
    section.SetYoungModulus(CABLE_YOUNG_MODULUS)
    section.SetDensity(CABLE_DENSITY)
    section.SetRayleighDamping(RAYLEIGH_DAMPING)
    KEEPALIVE.append(section)

    # === FEA mesh & cable === build the ANCF cable, then pin the rear node to the truss
    mesh = fea.ChMesh()
    mesh.SetAutomaticGravity(True)                   # gravity applied to FEA nodes
    KEEPALIVE.append(mesh)

    builder = fea.ChBuilderCableANCF()
    builder.BuildBeam(mesh, section, N_ELEMENTS, CABLE_START, CABLE_END)
    KEEPALIVE.append(builder)

    # SWIG GC pitfall: copy the node container into a strong-referenced list first
    beam_nodes_container = builder.GetLastBeamNodes()
    nodes = [beam_nodes_container[i] for i in range(beam_nodes_container.size())]
    KEEPALIVE.append(nodes)

    rear_node = nodes[0]                             # pinned end (at CABLE_START)
    front_node = nodes[-1]                           # free / loaded end (at CABLE_END)

    # Pin the rear node to the fixed truss with a node-frame constraint
    pin = fea.ChLinkNodeFrame()
    pin.Initialize(rear_node, truss)
    sys.Add(pin)
    KEEPALIVE.append(pin)

    # Apply the constant downward tip force on the free front node
    front_node.SetForce(TIP_FORCE)

    sys.Add(mesh)

    # === FEA visualization === color by node speed + an undeformed wireframe overlay
    vis_speed = chrono.ChVisualShapeFEA()
    vis_speed.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
    vis_speed.SetColormapRange(chrono.ChVector2d(0.0, 1.5))
    vis_speed.SetSmoothFaces(True)
    mesh.AddVisualShapeFEA(vis_speed)

    vis_wire = chrono.ChVisualShapeFEA()
    vis_wire.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
    vis_wire.SetWireframe(True)
    vis_wire.SetDrawInUndeformedReference(True)
    mesh.AddVisualShapeFEA(vis_wire)

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    vis = None
    if not HEADLESS:
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(sys)
        vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # gravity is along -Y here
        vis.SetWindowSize(1280, 720)
        vis.SetWindowTitle("ANCF cable - tip load (MINRES + HHT)")
        vis.Initialize()                                    # Initialize FIRST (inverse of VSG)
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddCamera(chrono.ChVector3d(0.5, 0.0, 1.6), chrono.ChVector3d(0.5, -0.3, 0.0))
        vis.AddTypicalLights()
        vis.AddGrid(0.1, 0.1, 30, 30,
                    chrono.ChCoordsysd(chrono.ChVector3d(0.5, -0.8, 0), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))          # ground reference grid

    # === Main loop === render-cadence outer loop; physics + CSV logging in inner batch
    os.makedirs("frames", exist_ok=True)                    # guard against missing output dir
    run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END    # short physics check when validating

    csv_file = None
    try:
        csv_file = open("simulation_data.csv", "w", newline="")  # context-managed close in finally
        writer = csv.writer(csv_file)
        writer.writerow(["time", "front_x", "front_y", "front_z",
                         "front_speed", "rear_y"])

        frame = 0
        while (HEADLESS or vis.Run()) and sys.GetChTime() < run_end:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index -> ffmpeg
                frame += 1
            for _ in range(RENDER_EVERY):
                t = sys.GetChTime()
                fpos = front_node.GetPos()              # free-end position this step
                fspeed = front_node.GetPosDt().Length()
                rpos = rear_node.GetPos()
                writer.writerow([f"{t:.5f}", f"{fpos.x:.6f}", f"{fpos.y:.6f}",
                                 f"{fpos.z:.6f}", f"{fspeed:.6f}", f"{rpos.y:.6f}"])
                sys.DoStepDynamics(TIME_STEP)
                if sys.GetChTime() >= run_end:
                    break
    except (RuntimeError, ValueError) as exc:   # solver divergence / invalid FEA state
        import traceback
        traceback.print_exc()
        raise
    except (OSError, IOError) as exc:           # disk / permission error opening/writing CSV
        import traceback
        traceback.print_exc()
        raise
    finally:
        if csv_file is not None:
            csv_file.close()                    # flush partial CSV even if a step diverged

    # === Post-processing === plot the logged time series to a PNG
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        times, front_y, front_speed = [], [], []
        with open("simulation_data.csv", "r", newline="") as f:   # context-managed read
            reader = csv.DictReader(f)
            for row in reader:
                times.append(float(row["time"]))
                front_y.append(float(row["front_y"]))
                front_speed.append(float(row["front_speed"]))

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
        ax1.plot(times, front_y, color="tab:blue")
        ax1.set_ylabel("front node Y (m)")
        ax1.set_title("ANCF cable free-end deflection")
        ax1.grid(True)
        ax2.plot(times, front_speed, color="tab:red")
        ax2.set_ylabel("front node speed (m/s)")
        ax2.set_xlabel("time (s)")
        ax2.grid(True)
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)
    except (OSError, ValueError) as exc:        # missing CSV / parse error during plotting
        import traceback
        traceback.print_exc()

    print(f"Done. nodes={len(nodes)} elements={N_ELEMENTS} final_time={sys.GetChTime():.3f}s")


if __name__ == "__main__":
    main()
