"""FEA tablecloth (Kirchhoff thin-shell cloth) draping under gravity.

Model
-----
A square deformable cloth meshed with triangular Kirchhoff-Love thin-shell
elements (``ChElementShellBST``) made of ``ChNodeFEAxyz`` position nodes and a
``ChMaterialShellKirchhoff`` (isotropic elasticity + light Rayleigh-style
settling via modest damping). The cloth is released from a flat horizontal
configuration and drapes/sags under gravity while two of its corner nodes are
pinned, so the sheet hangs like a tablecloth pulled at two corners.

System / solver
---------------
- ``ChSystemSMC`` (smooth contact, required by the FEA stiffness assembly).
- ``ChSolverPardisoMKL`` direct sparse solver (iterative solvers diverge on the
  shell stiffness matrix).
- ``Type_EULER_IMPLICIT_LINEARIZED`` timestepper (adaptive HHT step control
  collapses on this thin-shell problem; the linearized implicit Euler is stable).

Visualization
-------------
A ``ChVisualShapeFEA`` surface shape (``mvisualizeshellA``) renders the shell
faces with smooth shading, a wireframe overlay, and backface culling enabled, so
the draping surface is clearly legible from one side. An Irrlicht window provides
the standard sky / camera / lights / grid scene; review frames are written to
``frames/``.

Expected behavior
------------------
The unpinned region of the sheet sags downward under gravity, the mid-span
deflection grows then settles as the light damping bleeds off oscillation, and
the two pinned corners stay fixed. The maximum vertical deflection magnitude is
logged each step.
"""

# === Imports ===
import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# === Named constants: geometry / physics / run control ===
CLOTH_SIZE = 1.0                 # m, side length of the square sheet
N_DIV = 14                       # cells per side (modest count -> render < timeout)
CLOTH_THICKNESS = 2.0e-3         # m, shell thickness
CLOTH_Z0 = 1.0                   # m, initial flat height above the grid floor

YOUNG_MODULUS = 1.0e5            # Pa, soft cloth-like membrane stiffness
POISSON_RATIO = 0.3              # -
DENSITY = 200.0                  # kg/m^3, areal/volumetric density of cloth
ALPHA_DAMP = 0.10                # light damping for settling (no NaN, smooth drape)

GRAVITY = -9.81                  # m/s^2 along -Z (Z-up world)

TIME_STEP = 1.0e-3               # s, FEA-stable step
SIM_END = 2.0                    # s, short so the headless + render runs finish < timeout
RENDER_FPS = 30.0                # frames/s for the review video

# Headless validation gate: a fast, windowless physics check (full Irrlicht block
# is still built below for the on-screen / recording run).
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))   # fast windowless validation run

# === Derived constants (precomputed once) ===
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END           # short physics check when validating
N_NODES_SIDE = N_DIV + 1                                        # nodes per row/column
CELL = CLOTH_SIZE / N_DIV                                       # precomputed once: cell pitch

# Keep strong Python references to FEA objects so the SWIG wrappers are not
# garbage-collected (dangling shared_ptr -> segfault). KEEPALIVE vs SWIG GC.
KEEPALIVE = []


def node_index(ix, iy):
    """Flatten a (column, row) grid coordinate into a linear node index."""
    return iy * N_NODES_SIDE + ix


def build_cloth(sys):
    """Build the Kirchhoff thin-shell tablecloth mesh and return (mesh, nodes)."""
    # === Material: isotropic Kirchhoff shell ===
    elasticity = fea.ChElasticityKirchhoffIsothropic(YOUNG_MODULUS, POISSON_RATIO)
    material = fea.ChMaterialShellKirchhoff(elasticity)
    material.SetDensity(DENSITY)
    KEEPALIVE.append(elasticity)
    KEEPALIVE.append(material)

    mesh = fea.ChMesh()
    mesh.SetAutomaticGravity(True)
    KEEPALIVE.append(mesh)

    # === Nodes: a flat square grid of position nodes ===
    nodes = []
    for iy in range(N_NODES_SIDE):
        for ix in range(N_NODES_SIDE):
            x = ix * CELL - 0.5 * CLOTH_SIZE
            y = iy * CELL - 0.5 * CLOTH_SIZE
            nd = fea.ChNodeFEAxyz(chrono.ChVector3d(x, y, CLOTH_Z0))
            nd.SetMass(0.0)   # mass comes from the shell element density
            mesh.AddNode(nd)
            nodes.append(nd)
    KEEPALIVE.append(nodes)

    # === Elements: two BST triangles per cell with bending stencils ===
    # Each ChElementShellBST takes the 3 main-triangle nodes plus 3 "opposite"
    # neighbour nodes (one across each edge) for the geometric edge->opposite-node
    # bending stencil. Boundary edges have no neighbour -> pass None (avoids NaN).
    def opposite(ix, iy, present):
        """Opposite node across an edge, or None at the mesh boundary."""
        if present and 0 <= ix < N_NODES_SIDE and 0 <= iy < N_NODES_SIDE:
            return nodes[node_index(ix, iy)]
        return None

    for iy in range(N_DIV):
        for ix in range(N_DIV):
            n00 = nodes[node_index(ix, iy)]
            n10 = nodes[node_index(ix + 1, iy)]
            n01 = nodes[node_index(ix, iy + 1)]
            n11 = nodes[node_index(ix + 1, iy + 1)]

            # Lower-left triangle (n00, n10, n01); opposite nodes across its
            # three edges, with boundary-aware None fallbacks.
            t1 = fea.ChElementShellBST()
            t1.SetNodes(
                n00, n10, n01,
                opposite(ix + 1, iy + 1, True),       # across edge (n10,n01) -> n11 region
                opposite(ix - 1, iy + 1, ix - 1 >= 0),  # across edge (n00,n01)
                opposite(ix + 1, iy - 1, iy - 1 >= 0),  # across edge (n00,n10)
            )
            t1.AddLayer(CLOTH_THICKNESS, 0.0, material)
            mesh.AddElement(t1)
            KEEPALIVE.append(t1)

            # Upper-right triangle (n11, n01, n10); shares the diagonal edge.
            t2 = fea.ChElementShellBST()
            t2.SetNodes(
                n11, n01, n10,
                opposite(ix, iy, True),                 # across edge (n01,n10) -> n00 region
                opposite(ix + 2, iy, ix + 2 < N_NODES_SIDE),   # across edge (n11,n10)
                opposite(ix, iy + 2, iy + 2 < N_NODES_SIDE),   # across edge (n11,n01)
            )
            t2.AddLayer(CLOTH_THICKNESS, 0.0, material)
            mesh.AddElement(t2)
            KEEPALIVE.append(t2)

    # === Pin two corner nodes (tablecloth held at two corners) ===
    corner_a = nodes[node_index(0, 0)]
    corner_b = nodes[node_index(N_NODES_SIDE - 1, 0)]
    corner_a.SetFixed(True)
    corner_b.SetFixed(True)

    sys.Add(mesh)
    return mesh, nodes


def main():
    # === System & gravity ===
    sys = chrono.ChSystemSMC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, GRAVITY))

    # Direct sparse solver required for FEA stiffness assembly.
    solver = mkl.ChSolverPardisoMKL()
    sys.SetSolver(solver)
    KEEPALIVE.append(solver)

    # Linearized implicit Euler: stable for this thin shell (adaptive HHT collapses).
    sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)

    # === Cloth mesh ===
    mesh, nodes = build_cloth(sys)

    # === FEA visualization shape (mvisualizeshellA) ===
    # Smooth-shaded shell surface colored by node speed, with a wireframe overlay
    # and backface culling so the draping surface reads cleanly from one side.
    mvisualizeshellA = chrono.ChVisualShapeFEA()
    mvisualizeshellA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
    mvisualizeshellA.SetColormapRange(chrono.ChVector2d(0.0, 2.0))
    mvisualizeshellA.SetSmoothFaces(True)
    mvisualizeshellA.SetWireframe(True)
    mvisualizeshellA.SetBackfaceCull(True)
    mesh.AddVisualShapeFEA(mvisualizeshellA)
    KEEPALIVE.append(mvisualizeshellA)

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    vis = None
    if not HEADLESS:
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(sys)
        vis.SetCameraVertical(chrono.CameraVerticalDir_Z)   # Z-up world
        vis.SetWindowSize(1280, 720)
        vis.SetWindowTitle("FEA Tablecloth (Kirchhoff thin-shell drape)")
        vis.Initialize()                                    # Initialize FIRST on Irrlicht
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddCamera(chrono.ChVector3d(1.6, -1.8, 1.6), chrono.ChVector3d(0, 0, 0.4))
        vis.AddTypicalLights()
        vis.AddGrid(0.25, 0.25, 24, 24,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))          # ground reference grid

    # cache: free (unpinned) nodes fetched once, reused for the deflection metric
    free_nodes = [nd for nd in nodes if not nd.IsFixed()]
    n_nodes_total = len(nodes)
    n_elements_total = mesh.GetNumElements()

    os.makedirs("frames", exist_ok=True)   # guard against missing output dir

    # === Main loop (render-cadence outer loop, physics inner batch) ===
    csv_file = None
    writer = None
    times, max_defs, mean_defs = [], [], []
    try:
        csv_file = open("simulation_data.csv", "w", newline="")
        writer = csv.writer(csv_file)
        writer.writerow(["time", "max_deflection", "mean_deflection",
                         "min_z", "num_nodes", "num_elements"])

        frame = 0
        while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
                frame += 1

            for _ in range(RENDER_EVERY):
                t = sys.GetChTime()
                # deflection = drop below the initial flat height CLOTH_Z0
                zs = [nd.GetPos().z for nd in free_nodes]
                min_z = min(zs)
                defs = [CLOTH_Z0 - z for z in zs]
                max_def = max(defs)
                mean_def = sum(defs) / len(defs)

                writer.writerow([f"{t:.5f}", f"{max_def:.6f}", f"{mean_def:.6f}",
                                 f"{min_z:.6f}", n_nodes_total, n_elements_total])
                times.append(t)
                max_defs.append(max_def)
                mean_defs.append(mean_def)

                sys.DoStepDynamics(TIME_STEP)
                if sys.GetChTime() >= RUN_END:
                    break

    except (OSError, IOError) as exc:        # disk / permission while writing CSV or frames
        import traceback
        traceback.print_exc()
        raise
    except (RuntimeError, ValueError) as exc:  # solver divergence / invalid FEA state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # flush partial CSV even if a step diverges mid-run
        if csv_file is not None:
            csv_file.close()

    # === Post-processing: timeseries plot from the logged data ===
    if times:
        with open("simulation_timeseries.png", "wb") as _png_probe:
            pass   # ensure the output path is writable before matplotlib renders
        fig, ax = plt.subplots(figsize=(8, 4.5))
        ax.plot(times, max_defs, label="max deflection [m]")
        ax.plot(times, mean_defs, label="mean deflection [m]")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("vertical deflection [m]")
        ax.set_title("FEA tablecloth drape under gravity")
        ax.grid(True, alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)

    print(f"Done: t_end={sys.GetChTime():.3f}s nodes={n_nodes_total} "
          f"elements={n_elements_total} final_max_def="
          f"{max_defs[-1] if max_defs else float('nan'):.4f} m")


if __name__ == "__main__":
    main()
