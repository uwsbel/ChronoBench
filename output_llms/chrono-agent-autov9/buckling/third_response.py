"""Lateral buckling of a slender cantilever beam under a ramped compressive tip load.

Model
-----
A slender, vertical Euler-Bernoulli beam (PyChrono FEA, ChBuilderBeamEuler) is
clamped rigidly at its base and loaded at the free tip by an axial (downward,
along -Z) compressive force that is ramped up linearly in time. The beam is given
a tiny lateral imperfection seed (a small constant transverse force at the tip)
so that, once the compressive load passes the Euler critical buckling load, the
column loses lateral stability and deflects sideways (buckling) rather than
remaining perfectly straight.

System type
-----------
ChSystemSMC (FEA requires a smooth/penalty system). A direct Pardiso/MKL solver
is used because iterative solvers diverge on FEA stiffness matrices. The HHT
implicit timestepper integrates the stiff beam dynamics.

Main bodies
-----------
- A single FEA ChMesh holding one Euler beam discretized into several beam
  elements (ChNodeFEAxyzrot nodes). No rigid bodies, no contact: the column is
  driven purely by the clamped base constraint plus the prescribed tip loads.

Expected behavior / objective
------------------------------
While the ramped axial load is below the critical load the tip stays nearly on
the column axis; as the load crosses the critical value the lateral tip
deflection grows rapidly and visibly — the signature of column buckling. The
lateral tip deflection is logged to CSV and plotted versus time.
"""

# === Imports ===
import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless plotting backend
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# === Named constants (geometry / material / load / time) ===
BEAM_LENGTH = 2.0          # m, column height (clamped base -> free tip), along +Z
BEAM_DIAMETER = 0.04       # m, circular cross-section diameter (slender)
N_ELEMENTS = 12            # number of Euler beam elements along the column

YOUNG_MODULUS = 2.0e11     # Pa, steel
SHEAR_MODULUS = YOUNG_MODULUS * 0.35  # Pa, ~35% of E
DENSITY = 7800.0           # kg/m^3, steel
RAYLEIGH_DAMPING = 0.01    # structural (Rayleigh beta) damping

TIME_STEP = 5.0e-4         # s, small step for FEA stability
SIM_END = 4.0              # s, total simulated time
RENDER_FPS = 30.0          # frames per second for the review video

# Axial compressive tip load ramped linearly 0 -> AXIAL_LOAD_MAX over the run.
AXIAL_LOAD_MAX = 3.0e4     # N, peak downward (-Z) compressive force at the tip
                           # (set above the Euler critical load so the column
                           #  crosses the buckling threshold during the run)
LATERAL_IMPERFECTION = 5.0  # N, tiny constant +X seed force to break symmetry

# Euler critical buckling load for a fixed-free column: Pcr = pi^2 E I / (4 L^2).
# precomputed once (reported in CSV header comment) — diagnostic only.
_AREA_MOMENT_I = math.pi * (BEAM_DIAMETER ** 4) / 64.0  # precomputed once
P_CRITICAL = (math.pi ** 2) * YOUNG_MODULUS * _AREA_MOMENT_I / (4.0 * BEAM_LENGTH ** 2)

# Derived once — never recomputed in the loop.
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
LOAD_RATE = AXIAL_LOAD_MAX / SIM_END  # N/s, precomputed once: linear ramp slope

# Validation gate: a fast, windowless, short physics check.
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END  # short check when validating

# Strong references kept alive to avoid SWIG premature GC of FEA objects.
_keep_alive = {}


def main():
    # === System & gravity ===
    # FEA needs a smooth (penalty) system; gravity acts along -Z (Z-up world).
    sys = chrono.ChSystemSMC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    # === Solver & timestepper ===
    # Direct MKL/Pardiso solver: iterative solvers diverge on FEA stiffness.
    import pychrono.pardisomkl as mkl
    sys.SetSolver(mkl.ChSolverPardisoMKL())
    # HHT implicit integrator for the stiff beam. In this PyChrono 9.0.1 build the
    # generic timestepper getter does not expose HHT setters, so we simply select
    # the HHT type on the system (it integrates with its built-in defaults).
    sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)

    # === FEA mesh & beam (Euler-Bernoulli column) ===
    mesh = fea.ChMesh()
    mesh.SetAutomaticGravity(True)

    section = fea.ChBeamSectionEulerAdvanced()
    section.SetAsCircularSection(BEAM_DIAMETER)
    section.SetDensity(DENSITY)
    section.SetYoungModulus(YOUNG_MODULUS)
    section.SetShearModulus(SHEAR_MODULUS)
    section.SetRayleighDamping(RAYLEIGH_DAMPING)

    # FEA beam: no contact material needed — driven by the clamped base
    # constraint + prescribed tip forces + gravity only (it collides with nothing).
    builder = fea.ChBuilderBeamEuler()
    builder.BuildBeam(
        mesh, section, N_ELEMENTS,
        chrono.ChVector3d(0, 0, 0),             # base (clamped) at ground level
        chrono.ChVector3d(0, 0, BEAM_LENGTH),   # free tip
        chrono.ChVector3d(1, 0, 0),             # cross-section lateral reference
    )

    # Keep a strong reference to the node container BEFORE indexing (SWIG GC).
    beam_nodes_container = builder.GetLastBeamNodes()
    beam_nodes = [beam_nodes_container[i] for i in range(beam_nodes_container.size())]
    base_node = beam_nodes[0]   # cache: clamped base node, reused for asserts
    tip_node = beam_nodes[-1]   # cache: free tip node, queried every step

    # Clamp the base node fully (cantilever / fixed-free column).
    base_node.SetFixed(True)

    sys.Add(mesh)

    # Keep FEA objects alive past function locals.
    _keep_alive["mesh"] = mesh
    _keep_alive["section"] = section
    _keep_alive["builder"] = builder
    _keep_alive["nodes"] = beam_nodes

    # Sanity: tip starts on the column axis at the top.
    p0 = tip_node.GetPos()
    assert abs(p0.x) < 1e-6 and abs(p0.z - BEAM_LENGTH) < 1e-6, "tip not at column top"

    # === FEA visualization shapes ===
    # Colored deformed mesh by node speed + an undeformed wireframe reference.
    vis_beam = chrono.ChVisualShapeFEA()
    vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_DISP_NORM)
    vis_beam.SetColormapRange(chrono.ChVector2d(0.0, 0.3))
    vis_beam.SetSmoothFaces(True)
    mesh.AddVisualShapeFEA(vis_beam)

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
        vis.SetWindowSize(1280, 720)
        vis.SetWindowTitle("Cantilever column buckling under ramped tip load")
        vis.SetCameraVertical(chrono.CameraVerticalDir_Z)  # Z-up world
        vis.Initialize()  # Initialize FIRST, then add scene elements (Irrlicht order)
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddCamera(chrono.ChVector3d(3.5, -3.5, 1.5),
                      chrono.ChVector3d(0, 0, BEAM_LENGTH * 0.5))
        vis.AddTypicalLights()
        vis.AddGrid(0.25, 0.25, 24, 24,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))  # ground reference grid

    # === Output setup ===
    os.makedirs("frames", exist_ok=True)  # guard against missing output dir
    os.makedirs("cam", exist_ok=True)     # review-video frame container

    csv_file = None
    try:
        csv_file = open("simulation_data.csv", "w", newline="")
    except (OSError, IOError) as exc:  # disk full / permission denied
        print("Could not open CSV for writing:", exc)
        raise

    times, axial_loads, lateral_defs = [], [], []

    # === Main loop (render-cadence outer loop, physics inner batch) ===
    writer = csv.writer(csv_file)
    writer.writerow(["time_s", "axial_load_N", "lateral_tip_deflection_m",
                     "tip_x_m", "tip_z_m"])
    writer.writerow(["# P_critical_N", f"{P_CRITICAL:.3f}", "", "", ""])

    frame = 0
    try:
        while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index
                frame += 1

            for _ in range(RENDER_EVERY):
                t = sys.GetChTime()
                # Ramp the compressive (-Z) axial load; keep a tiny +X seed.
                axial = LOAD_RATE * t
                tip_node.SetForce(chrono.ChVector3d(
                    LATERAL_IMPERFECTION, 0.0, -axial))

                # Log this step's lateral tip deflection.
                tp = tip_node.GetPos()
                lateral = math.hypot(tp.x, tp.y)
                times.append(t)
                axial_loads.append(axial)
                lateral_defs.append(lateral)
                writer.writerow([f"{t:.6f}", f"{axial:.4f}", f"{lateral:.8f}",
                                 f"{tp.x:.8f}", f"{tp.z:.8f}"])

                sys.DoStepDynamics(TIME_STEP)
                if sys.GetChTime() >= RUN_END:
                    break
    except (RuntimeError, ValueError) as exc:  # solver divergence / invalid state
        import traceback
        traceback.print_exc()
        print("Simulation aborted:", exc)
        raise
    finally:
        # Flush + close the CSV even if a step diverged mid-run.
        if csv_file is not None:
            csv_file.flush()
            csv_file.close()

    # === Post-processing (timeseries plot) ===
    if times:
        fig, ax1 = plt.subplots(figsize=(8, 5))
        ax1.plot(times, lateral_defs, "b-", label="lateral tip deflection [m]")
        ax1.set_xlabel("time [s]")
        ax1.set_ylabel("lateral tip deflection [m]", color="b")
        ax1.tick_params(axis="y", labelcolor="b")

        ax2 = ax1.twinx()
        ax2.plot(times, axial_loads, "r--", label="axial load [N]")
        ax2.axhline(P_CRITICAL, color="k", linestyle=":",
                    label=f"P_critical = {P_CRITICAL:.0f} N")
        ax2.set_ylabel("axial compressive load [N]", color="r")
        ax2.tick_params(axis="y", labelcolor="r")

        fig.suptitle("Cantilever column buckling: lateral deflection vs ramped load")
        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper left")
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)

        print(f"Steps logged: {len(times)}  P_critical={P_CRITICAL:.1f} N  "
              f"final lateral deflection={lateral_defs[-1]:.5f} m  "
              f"max axial load={axial_loads[-1]:.1f} N")


if __name__ == "__main__":
    main()
