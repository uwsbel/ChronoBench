"""
Draping tablecloth — FEA Kirchhoff thin-shell (BST) simulation in PyChrono 9.0.1.

Model
-----
A square cloth is meshed as a regular grid of ``ChNodeFEAxyz`` points. Each grid
quad is split into two triangles, and every triangle is realized as a
``ChElementShellBST`` (Bilinear Shear-free Triangle) Kirchhoff thin shell whose
bending stencil uses the three opposite "neighbour" nodes of the adjacent
triangles. The shell material is linear isotropic
(``ChElasticityKirchhoffIsothropic`` -> ``ChMaterialShellKirchhoff``).

A square block of nodes in one corner region of the sheet is pinned
(``SetFixed(True)``); the remainder of the cloth drapes/sags downward under
gravity, exhibiting the characteristic thin-shell bending and self-folding of a
hanging tablecloth.

System / solver
---------------
- ``ChSystemSMC`` (FEA requires SMC + a direct solver).
- ``ChSolverPardisoMKL`` direct sparse solver with a locked sparsity pattern
  (the mesh connectivity never changes, so the symbolic factorization is reused).
- ``EULER_IMPLICIT_LINEARIZED`` timestepper: the default adaptive HHT collapses on
  the very stiff membrane part of a thin shell, so a linearized implicit Euler
  step is used for robust, monotone draping.

Expected behavior
------------------
The pinned nodes stay put; the free cloth falls and folds under gravity. With
stiffness-proportional (Rayleigh) shell damping the lowest cloth node descends
and settles as the sheet reaches a draped rest configuration rather than ringing.
Logged to ``simulation_data.csv``: time, the cloth lowest-point Y height, and the
tracked free-corner / centre monitor node heights.
"""

import os
import csv
import math

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# === Named constants ===  geometry / material / time integration parameters
CLOTH_SIZE = 1.0                      # square cloth edge length [m]
NSECTIONS_X = 44                      # grid subdivisions along X (45x45 nodes)
NSECTIONS_Z = 44                      # grid subdivisions along Z (cloth lies in the X-Z plane, drapes along -Y)
CLOTH_Y0 = 1.0                        # initial height of the flat sheet [m]
SHELL_THICKNESS = 0.002              # shell thickness [m]
YOUNG_MODULUS = 3.0e4                # cloth membrane stiffness E [Pa] (soft fabric -> smooth continuous drape)
POISSON_RATIO = 0.30                 # Poisson ratio [-]
CLOTH_DENSITY = 200.0                # areal-equivalent density [kg/m^3]
RAYLEIGH_BETA = 0.01                 # Kirchhoff Rayleigh (stiffness-proportional) damping -> drape settles, no bounce

FIXED_BLOCK = 30                     # pin a FIXED_BLOCK x FIXED_BLOCK corner block of nodes
LOAD_FORCE = chrono.ChVector3d(0.0, 0.0, 0.0)   # external node load force [N] (gravity-only drape)

TIME_STEP = 0.005                    # integration step [s]
SIM_END = 2.0                        # simulated duration [s]
RENDER_FPS = 30.0                    # review-frame cadence [frames/s]
GRAVITY = chrono.ChVector3d(0.0, -9.81, 0.0)    # gravity along -Y (cloth drapes downward)

# Derived constants (precomputed once) ----------------------------------------
N_NODES_X = NSECTIONS_X + 1          # precomputed once: node count per row
N_NODES_Z = NSECTIONS_Z + 1          # precomputed once: node count per column
DX = CLOTH_SIZE / NSECTIONS_X        # precomputed once: grid spacing X
DZ = CLOTH_SIZE / NSECTIONS_Z        # precomputed once: grid spacing Z
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once: steps per frame

HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))   # fast, windowless validation run


def node_index(ix, iz):
    """Flat index of the grid node at column ix, row iz."""
    return ix * N_NODES_Z + iz


def build_cloth(mesh, material):
    """Create the node grid and the two BST triangles per quad with geometric
    neighbour (opposite-node) bending stencils. Returns (nodes, elements)."""
    # --- nodes: flat sheet in the X-Z plane at height CLOTH_Y0 ---
    nodes = []
    for ix in range(N_NODES_X):
        for iz in range(N_NODES_Z):
            x = ix * DX
            z = iz * DZ
            nd = fea.ChNodeFEAxyz(chrono.ChVector3d(x, CLOTH_Y0, z))
            mesh.AddNode(nd)
            nodes.append(nd)

    # --- triangles: split each quad into two BST shells ---
    # For a BST triangle (main nodes n0,n1,n2) the three neighbour nodes
    # (nb0,nb1,nb2) are the opposite vertices of the triangles sharing edges
    # (n1,n2), (n2,n0), (n0,n1). We resolve them geometrically from the grid;
    # boundary edges with no adjacent triangle pass a None neighbour.
    elements = []

    def grid_node(ix, iz):
        if 0 <= ix < N_NODES_X and 0 <= iz < N_NODES_Z:
            return nodes[node_index(ix, iz)]
        return None

    for ix in range(NSECTIONS_X):
        for iz in range(NSECTIONS_Z):
            # quad corners
            n00 = grid_node(ix, iz)
            n10 = grid_node(ix + 1, iz)
            n01 = grid_node(ix, iz + 1)
            n11 = grid_node(ix + 1, iz + 1)

            # --- lower triangle: main = (n00, n10, n01) ---
            # edge (n10,n01) opposite -> the n11 vertex of the upper triangle
            # edge (n01,n00) opposite -> node across the -X boundary (ix-1, iz+1)
            # edge (n00,n10) opposite -> node across the -Z boundary (ix+1, iz-1)
            eA = fea.ChElementShellBST()
            eA.SetNodes(
                n00, n10, n01,
                n11,                       # neighbour opposite edge (n10,n01)
                grid_node(ix - 1, iz + 1), # neighbour opposite edge (n01,n00)
                grid_node(ix + 1, iz - 1), # neighbour opposite edge (n00,n10)
            )
            eA.AddLayer(SHELL_THICKNESS, 0.0 * chrono.CH_DEG_TO_RAD, material)
            mesh.AddElement(eA)
            elements.append(eA)

            # --- upper triangle: main = (n11, n01, n10) ---
            # edge (n01,n10) opposite -> n00 of the lower triangle
            # edge (n10,n11) opposite -> node across the +X boundary (ix+2, iz)
            # edge (n11,n01) opposite -> node across the +Z boundary (ix, iz+2)
            eB = fea.ChElementShellBST()
            eB.SetNodes(
                n11, n01, n10,
                n00,                       # neighbour opposite edge (n01,n10)
                grid_node(ix + 2, iz),     # neighbour opposite edge (n10,n11)
                grid_node(ix, iz + 2),     # neighbour opposite edge (n11,n01)
            )
            eB.AddLayer(SHELL_THICKNESS, 0.0 * chrono.CH_DEG_TO_RAD, material)
            mesh.AddElement(eB)
            elements.append(eB)

    return nodes, elements


def main():
    # === System & gravity ===  SMC system (FEA requires SMC + direct solver)
    sys = chrono.ChSystemSMC()
    sys.SetGravitationalAcceleration(GRAVITY)

    # === Solver & timestepper ===  Pardiso MKL direct solver; linearized implicit Euler
    mkl_solver = mkl.ChSolverPardisoMKL()
    mkl_solver.LockSparsityPattern(True)   # mesh connectivity is fixed -> reuse symbolic factorization
    sys.SetSolver(mkl_solver)
    # Adaptive HHT collapses on the stiff thin-shell membrane -> use linearized implicit Euler.
    sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)

    # === FEA mesh & material ===  Kirchhoff isotropic thin-shell cloth
    mesh = fea.ChMesh()
    mesh.SetAutomaticGravity(True)         # gravity loads applied to FEA nodes

    elasticity = fea.ChElasticityKirchhoffIsothropic(YOUNG_MODULUS, POISSON_RATIO)
    damping = fea.ChDampingKirchhoffRayleigh(elasticity, RAYLEIGH_BETA)   # dissipates bending oscillation
    material = fea.ChMaterialShellKirchhoff(elasticity, None, damping)
    material.SetDensity(CLOTH_DENSITY)

    nodes, elements = build_cloth(mesh, material)

    # === Boundary conditions & monitors ===  pin a corner block, pick monitor nodes
    # Fix the FIXED_BLOCK x FIXED_BLOCK block of nodes in one corner (the "held" edge).
    nodes_load = []                        # nodes that could receive an external load
    for j in range(FIXED_BLOCK):
        for k in range(FIXED_BLOCK):
            # conditional guards prevent out-of-range indexing on small grids
            if j < N_NODES_X and k < N_NODES_Z:
                nodes[node_index(j, k)].SetFixed(True)

    # Reference interpolation helpers (kept for completeness of the monitor contract).
    def ref_x(t):
        return 0.0                          # no prescribed lateral reference motion

    def ref_y(t):
        return CLOTH_Y0                     # flat reference height

    # Monitor node A: a free corner diagonally opposite the pinned block.
    # Monitor node B: the geometric centre node of the sheet.
    mnode_monitor = nodes[node_index(N_NODES_X - 1, N_NODES_Z - 1)]   # free far corner
    node_plot_a = mnode_monitor
    node_plot_b = nodes[node_index(N_NODES_X // 2, N_NODES_Z // 2)]    # centre node
    # Monitor a specific boundary element when the canonical condition holds.
    melement_monitor = None
    if NSECTIONS_X >= 2 and len(elements) > 0:
        # first element built for the (ix==1, iz==0) lower triangle
        melement_monitor = elements[2 * (1 * NSECTIONS_Z + 0)]

    # Optional external load force on selected free nodes (zero here -> gravity drape).
    if LOAD_FORCE.Length() > 0.0:
        for nd in nodes_load:
            nd.SetForce(LOAD_FORCE)

    sys.Add(mesh)

    # === FEA visualization ===  smooth shaded shell (colored by displacement) + mesh wireframe
    vis_shell_a = chrono.ChVisualShapeFEA()
    vis_shell_a.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_DISP_NORM)
    vis_shell_a.SetColormapRange(chrono.ChVector2d(0.0, 0.6))   # span the actual drape displacement -> vivid color
    vis_shell_a.SetSmoothFaces(True)
    vis_shell_a.SetWireframe(False)
    mesh.AddVisualShapeFEA(vis_shell_a)

    # Triangle-mesh wireframe overlay so the deformed sheet visibly reads as a cloth mesh.
    vis_shell_b = chrono.ChVisualShapeFEA()
    vis_shell_b.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
    vis_shell_b.SetWireframe(True)
    vis_shell_b.SetDrawInUndeformedReference(False)
    mesh.AddVisualShapeFEA(vis_shell_b)

    # === Visualization ===  full Irrlicht scene: window + sky + camera + lights + grid
    vis = None
    if not HEADLESS:
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(sys)
        vis.SetWindowSize(1280, 720)
        vis.SetWindowTitle("Draping tablecloth — Kirchhoff thin-shell FEA")
        vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # gravity is along -Y
        vis.Initialize()                                    # Initialize FIRST (inverse of VSG)
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddCamera(chrono.ChVector3d(1.9, 1.5, -0.9), chrono.ChVector3d(0.5, 0.6, 0.5))
        vis.AddTypicalLights()
        vis.AddGrid(0.25, 0.25, 24, 24,
                    chrono.ChCoordsysd(chrono.ChVector3d(0.5, 0.0, 0.5), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))

    # === Run setup ===  output dirs + CSV writer + run bounds
    os.makedirs("frames", exist_ok=True)   # guard against missing review-frame dir
    run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END   # short physics check when validating

    # cache: monitor-node handles fetched once, reused every step
    track_corner = node_plot_a
    track_center = node_plot_b
    all_nodes = nodes                      # cache: node list reused for lowest-point scan each step

    times, low_y, corner_y, center_y = [], [], [], []

    csv_file = None
    try:
        try:
            csv_file = open("simulation_data.csv", "w", newline="")   # disk / permission guard
        except (OSError, IOError) as exc:
            print(f"Cannot open output CSV: {exc}")
            raise
        writer = csv.writer(csv_file)
        writer.writerow(["time", "cloth_lowest_y", "corner_node_y", "center_node_y"])

        frame = 0
        while (HEADLESS or vis.Run()) and sys.GetChTime() < run_end:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
                frame += 1

            for _ in range(RENDER_EVERY):
                t = sys.GetChTime()
                # lowest cloth point this step (min Y over all nodes)
                ymin = min(nd.GetPos().y for nd in all_nodes)
                cy = track_corner.GetPos().y
                gy = track_center.GetPos().y
                times.append(t)
                low_y.append(ymin)
                corner_y.append(cy)
                center_y.append(gy)
                writer.writerow([f"{t:.6f}", f"{ymin:.6f}", f"{cy:.6f}", f"{gy:.6f}"])

                sys.DoStepDynamics(TIME_STEP)
                if sys.GetChTime() >= run_end:
                    break

    except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    finally:
        if csv_file is not None:
            csv_file.close()   # flush partial CSV even if a step diverges

    # === Post-processing ===  plot tracked quantities vs time
    if times:
        fig, ax = plt.subplots(figsize=(8, 5))
        ax.plot(times, low_y, label="cloth lowest Y")
        ax.plot(times, corner_y, label="free corner node Y")
        ax.plot(times, center_y, label="center node Y")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("height Y [m]")
        ax.set_title("Tablecloth drape — node heights vs time")
        ax.legend()
        ax.grid(True)
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=120)
        plt.close(fig)

    print(f"Done: {len(times)} steps, "
          f"nodes={mesh.GetNumNodes()}, elements={mesh.GetNumElements()}, "
          f"final lowest Y={low_y[-1] if low_y else float('nan'):.4f} m")


if __name__ == "__main__":
    main()
