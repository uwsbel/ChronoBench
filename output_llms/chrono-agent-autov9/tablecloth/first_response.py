"""Tablecloth folding/draping simulation using thin-shell finite elements.

Models a square tablecloth as a mesh of Kirchhoff thin-shell triangle elements
(ChElementShellBST) built on a regular grid of position-only FEA nodes
(ChNodeFEAxyz). The cloth is given an *isotropic* Kirchhoff elasticity law
(ChElasticityKirchhoffIsothropic) wrapped in a ChMaterialShellKirchhoff. The four
corner nodes are pinned; under gravity the unconstrained interior nodes sag and the
cloth folds/drapes between the pinned corners.

System type: ChSystemSMC (required by Chrono FEA — implicit stiffness assembly).
Solver:      PardisoMKL direct sparse solver (efficient/robust on FEA stiffness).
Integrator:  Linearized implicit Euler (fixed-step) — stable for stiff shell dynamics.
Main bodies: one ChMesh holding NxN FEA nodes and the BST shell elements.
Expected behavior: the pinned-corner tablecloth releases from a flat horizontal
configuration and drapes downward in the middle, folding into a sagging surface;
the lowest cloth point descends monotonically then settles as the cloth tautens.
"""

import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

# === Named constants === geometry / physics / discretization (no bare literals downstream)
CLOTH_SIZE = 1.0           # m, side length of the square tablecloth
GRID_N = 14                # nodes per side -> (GRID_N-1)^2 quads, 2 tris each
CLOTH_HEIGHT = 1.0         # m, initial height of the flat cloth above the ground grid
THICKNESS = 0.002          # m, cloth shell thickness (2 mm fabric)
YOUNG_MODULUS = 1.0e6      # Pa, isotropic Kirchhoff Young's modulus (soft fabric)
POISSON = 0.3              # isotropic Kirchhoff Poisson ratio
DENSITY = 200.0            # kg/m^3, fabric density
RAYLEIGH_BETA = 0.04       # stiffness-proportional damping for shell stability
GRAVITY = -9.81            # m/s^2 along world Z

TIME_STEP = 1.0e-3         # s, FEA-stable implicit step
SIM_END = 2.5              # s, long enough to see the cloth drape and settle
RENDER_FPS = 30.0          # review-video frame rate

# derived constants (precomputed once — never recomputed in the loop)
SPACING = CLOTH_SIZE / (GRID_N - 1)          # precomputed once: node pitch
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once: steps/frame

HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))  # fast windowless validation run
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END  # short physics check when validating

# Keep strong Python references against SWIG garbage collection (nodes/elements/mats).
KEEPALIVE = []


def node_index(ix, iy):
    """Flat index of grid node (ix, iy) in row-major order."""
    return iy * GRID_N + ix


def main():
    # === System & gravity === SMC system is required for Chrono FEA assembly
    sys = chrono.ChSystemSMC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, GRAVITY))

    # === Solver & timestepper === PardisoMKL direct solver + implicit integrator
    solver = mkl.ChSolverPardisoMKL()          # direct sparse solver, robust on FEA stiffness
    sys.SetSolver(solver)
    KEEPALIVE.append(solver)
    # Linearized implicit Euler: fixed-step, no adaptive step-control — stable for the
    # stiff thin-shell stiffness matrix (adaptive HHT collapses to min step here).
    sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)

    # === Material === isotropic Kirchhoff thin-shell elasticity law
    elasticity = fea.ChElasticityKirchhoffIsothropic(YOUNG_MODULUS, POISSON)
    damping = fea.ChDampingKirchhoffRayleigh(elasticity, RAYLEIGH_BETA)
    material = fea.ChMaterialShellKirchhoff(elasticity, None, damping)
    material.SetDensity(DENSITY)
    KEEPALIVE.extend([elasticity, damping, material])

    # === FEA mesh: nodes === flat NxN grid of position-only nodes at CLOTH_HEIGHT
    mesh = fea.ChMesh()
    mesh.SetAutomaticGravity(True)             # gravity applied to shell elements
    KEEPALIVE.append(mesh)

    nodes = []                                 # cache: all node handles, indexed via node_index
    origin = -0.5 * CLOTH_SIZE                 # precomputed once: center the cloth on origin
    for iy in range(GRID_N):
        for ix in range(GRID_N):
            px = origin + ix * SPACING
            py = origin + iy * SPACING
            nd = fea.ChNodeFEAxyz(chrono.ChVector3d(px, py, CLOTH_HEIGHT))
            mesh.AddNode(nd)
            nodes.append(nd)
    KEEPALIVE.append(nodes)

    # Pin the four corner nodes so the cloth folds/drapes between them.
    corner_indices = [
        node_index(0, 0),
        node_index(GRID_N - 1, 0),
        node_index(0, GRID_N - 1),
        node_index(GRID_N - 1, GRID_N - 1),
    ]
    for ci in corner_indices:
        nodes[ci].SetFixed(True)

    # === FEA mesh: BST shell elements === split each grid quad into two triangles
    # ChElementShellBST = Kirchhoff thin-shell triangle: 3 main nodes (n0,n1,n2) plus
    # up to 3 neighbour nodes used for cross-edge bending. Neighbour k is the node of
    # the adjacent triangle that lies OPPOSITE the edge connecting the two main nodes
    # other than node k:  nb0 ~ edge(n1,n2), nb1 ~ edge(n2,n0), nb2 ~ edge(n0,n1).
    # Boundary edges with no adjacent triangle pass None.
    triangles = []                              # list of (a, b, c) main-node triples
    for iy in range(GRID_N - 1):
        for ix in range(GRID_N - 1):
            bl = node_index(ix, iy)
            br = node_index(ix + 1, iy)
            tl = node_index(ix, iy + 1)
            tr = node_index(ix + 1, iy + 1)
            triangles.append((bl, br, tr))      # lower-right triangle
            triangles.append((bl, tr, tl))      # upper-left triangle

    # Map each undirected edge -> the opposite node(s) of the triangle(s) using it.
    edge_opposite = {}                          # cache: built once before element creation
    for (a, b, c) in triangles:
        for (u, v, w) in ((a, b, c), (b, c, a), (c, a, b)):
            key = (min(u, v), max(u, v))        # edge (u,v); opposite node is w
            edge_opposite.setdefault(key, []).append(w)

    def opposite_across(edge_u, edge_v, exclude):
        """Neighbour node across edge (edge_u, edge_v), excluding this triangle's own node."""
        key = (min(edge_u, edge_v), max(edge_u, edge_v))
        for cand in edge_opposite.get(key, ()):
            if cand != exclude:
                return cand
        return None

    for (n0, n1, n2) in triangles:
        nb0 = opposite_across(n1, n2, n0)       # opposite edge (n1,n2)
        nb1 = opposite_across(n2, n0, n1)       # opposite edge (n2,n0)
        nb2 = opposite_across(n0, n1, n2)       # opposite edge (n0,n1)
        el = fea.ChElementShellBST()
        el.SetNodes(
            nodes[n0], nodes[n1], nodes[n2],
            None if nb0 is None else nodes[nb0],
            None if nb1 is None else nodes[nb1],
            None if nb2 is None else nodes[nb2],
        )
        el.AddLayer(THICKNESS, 0.0 * chrono.CH_DEG_TO_RAD, material)  # single isotropic layer
        el.SetLayerZreferenceCentered()         # mid-surface reference for the layer
        mesh.AddElement(el)
        KEEPALIVE.append(el)

    sys.Add(mesh)

    # === FEA visualization shapes === colored draped surface + wireframe overlay
    vis_surface = chrono.ChVisualShapeFEA()
    vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
    vis_surface.SetColormapRange(chrono.ChVector2d(0.0, 1.5))
    vis_surface.SetSmoothFaces(True)
    mesh.AddVisualShapeFEA(vis_surface)
    vis_wire = chrono.ChVisualShapeFEA()
    vis_wire.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
    vis_wire.SetWireframe(True)
    mesh.AddVisualShapeFEA(vis_wire)
    KEEPALIVE.extend([vis_surface, vis_wire])

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    vis = None
    if not HEADLESS:
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(sys)
        vis.SetCameraVertical(chrono.CameraVerticalDir_Z)   # gravity along -Z
        vis.SetWindowSize(1280, 720)
        vis.SetWindowTitle("Tablecloth folding — Kirchhoff thin-shell FEA")
        vis.Initialize()                                    # Initialize FIRST (inverse of VSG)
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddCamera(chrono.ChVector3d(1.8, -1.8, 1.6), chrono.ChVector3d(0, 0, 0.6))
        vis.AddTypicalLights()
        vis.AddGrid(0.25, 0.25, 24, 24,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))          # ground reference grid

    # === Output setup === relative paths resolve against the script directory
    os.makedirs("frames", exist_ok=True)       # guard against missing frame dir
    os.makedirs("cam", exist_ok=True)          # guard against missing video dir

    # cache: track the cloth center node and total node count once, reuse every step
    center_node = nodes[node_index(GRID_N // 2, GRID_N // 2)]  # cache: fetched once
    n_nodes = len(nodes)                                       # precomputed once

    times, center_z, lowest_z, mean_z = [], [], [], []

    csv_file = None
    try:
        csv_file = open("simulation_data.csv", "w", newline="")  # context-managed below
    except (OSError, IOError) as exc:          # disk full / permission error on open
        print(f"Could not open CSV output: {exc}")
        raise

    # === Main loop === render-cadence outer loop; physics + logging in inner batch
    try:
        with csv_file as f:
            writer = csv.writer(f)
            writer.writerow(["time", "center_z", "lowest_z", "mean_z"])

            frame = 0
            while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
                if not HEADLESS:
                    vis.BeginScene()
                    vis.Render()
                    vis.EndScene()
                    vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index
                    frame += 1
                for _ in range(RENDER_EVERY):
                    t = sys.GetChTime()
                    # sample cloth geometry each physics step
                    zc = center_node.GetPos().z
                    zmin = math.inf
                    zsum = 0.0
                    for nd in nodes:
                        z = nd.GetPos().z
                        if z < zmin:
                            zmin = z
                        zsum += z
                    times.append(t)
                    center_z.append(zc)
                    lowest_z.append(zmin)
                    mean_z.append(zsum / n_nodes)
                    writer.writerow([f"{t:.5f}", f"{zc:.6f}", f"{zmin:.6f}", f"{zsum / n_nodes:.6f}"])

                    sys.DoStepDynamics(TIME_STEP)
                    if sys.GetChTime() >= RUN_END:
                        break
    except (RuntimeError, ValueError) as exc:  # solver divergence / invalid FEA state
        import traceback
        traceback.print_exc()
        print(f"Simulation aborted: {exc}")
        raise
    finally:
        # CSV is flushed/closed by the `with` block; report progress regardless.
        print(f"Logged {len(times)} steps; final center_z="
              f"{center_z[-1] if center_z else float('nan'):.4f} m")

    # === Post-processing === plot the draping geometry vs time
    if times:
        with open("simulation_data.csv", "r") as _check:  # context-managed sanity read
            assert _check.readline().strip() != "", "CSV header missing"

        fig, ax = plt.subplots(figsize=(8, 5))
        ax.plot(times, center_z, label="center node z")
        ax.plot(times, lowest_z, label="lowest cloth point z")
        ax.plot(times, mean_z, label="mean cloth z")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("height z [m]")
        ax.set_title("Tablecloth draping under gravity (Kirchhoff shell FEA)")
        ax.legend()
        ax.grid(True)
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=120)
        plt.close(fig)


if __name__ == "__main__":
    main()
