"""Folding of a tablecloth modeled with Kirchhoff thin-shell finite elements.

Model
-----
A square tablecloth is discretized as a regular grid of `ChNodeFEAxyz` nodes and
meshed with triangular `ChElementShellBST` Kirchhoff-Love thin-shell elements
(Bending-STrip / hinge formulation). The fabric is an *isotropic* Kirchhoff
material (`ChElasticityKirchhoffIsothropic`, Young modulus + Poisson ratio) with a
small per-area mass. Two diagonally-opposite corners of the cloth are pinned
(`SetFixed`) while gravity pulls the unsupported span down, so the sheet sags and
folds under its own weight — the classic draping / folding behavior.

System / numerics
-----------------
- `ChSystemSMC` (smooth contact system; required for the FEA stiffness path) with
  the PardisoMKL direct solver — iterative solvers diverge on shell stiffness.
- `EULER_IMPLICIT_LINEARIZED` timestepper: a stiff thin shell makes the adaptive
  HHT step controller collapse, so a fixed linearized implicit Euler step is used.
- Pure FEA shell driven by gravity + pinned corners only: there is NO rigid-body
  contact in the scene, so no collision system / contact material is created.

Visualization is an Irrlicht window with a `ChVisualShapeFEA` colored by node
displacement plus a wireframe overlay. Expected behavior: the cloth droops between
the two pinned corners and develops folds, settling toward a catenary-like drape.
"""

import os
import math

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl


# === Parameters === geometry, material, and time integration constants
cloth_size = 1.0            # side length of the square tablecloth [m]
n_div = 16                  # grid subdivisions per side -> (n_div+1)^2 nodes
thickness = 2.0e-3          # shell thickness [m]
youngs_modulus = 1.0e6      # isotropic Kirchhoff Young modulus [Pa] (soft fabric)
poisson_ratio = 0.3         # isotropic Poisson ratio [-]
density = 500.0             # material density [kg/m^3]
rayleigh_damping = 0.01     # Rayleigh beta damping for the shell material
start_height = 0.6          # initial height of the flat cloth above the grid [m]

time_step = 1.0e-3          # fixed integration step [s]
sim_end = 3.0               # total simulated time [s]
render_fps = 50.0           # review render cadence [frames/s]
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

n_nodes_side = n_div + 1
step_len = cloth_size / n_div          # precomputed once: grid spacing [m]

# === System & gravity === SMC system with the direct MKL solver for FEA shells
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
# Pure FEA thin shell pinned at corners + gravity only: NO rigid-body contact in
# this scene, so no collision system or contact material is defined.
solver = mkl.ChSolverPardisoMKL()
sys.SetSolver(solver)
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)

# === FEA material === isotropic Kirchhoff thin-shell fabric
elasticity = fea.ChElasticityKirchhoffIsothropic(youngs_modulus, poisson_ratio)
damping = fea.ChDampingKirchhoffRayleigh(elasticity, rayleigh_damping)
material = fea.ChMaterialShellKirchhoff(elasticity, None, damping)
material.SetDensity(density)

# === FEA mesh === node grid + BST triangular shell elements
# Strong references kept in lists/dicts so SWIG does not GC the shared_ptrs.
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

keepalive = {"mesh": mesh, "material": material, "elasticity": elasticity,
             "damping": damping}   # cache: hold refs so SWIG GC cannot dangle them

# Build the (n_nodes_side x n_nodes_side) node grid in the XY plane, lifted to
# start_height in Z so the flat sheet drapes downward under gravity.
nodes = []
for iy in range(n_nodes_side):
    row = []
    for ix in range(n_nodes_side):
        px = -0.5 * cloth_size + ix * step_len
        py = -0.5 * cloth_size + iy * step_len
        node = fea.ChNodeFEAxyz(chrono.ChVector3d(px, py, start_height))
        mesh.AddNode(node)
        row.append(node)
    nodes.append(row)
keepalive["nodes"] = nodes


def grid_node(ix, iy):
    """Return the grid node at (ix, iy) or None if outside the grid (boundary)."""
    if 0 <= ix < n_nodes_side and 0 <= iy < n_nodes_side:
        return nodes[iy][ix]
    return None


# Each grid quad is split into two BST triangles (A and B). The first three nodes
# are the main triangle; nodes 3,4,5 are the bending-hinge neighbours that sit
# ACROSS the three triangle edges in the adjacent quad. The neighbour index is
# derived GEOMETRICALLY from the grid (the opposite grid node across each edge),
# and is None whenever that edge lies on the cloth boundary — passing a degenerate
# / wrong neighbour instead injects NaN into the Kirchhoff bending assembly.
elements = []
for iy in range(n_div):
    for ix in range(n_div):
        # Element A: main triangle (ix,iy), (ix+1,iy), (ix,iy+1)
        a0 = grid_node(ix, iy)
        a1 = grid_node(ix + 1, iy)
        a2 = grid_node(ix, iy + 1)
        a3 = grid_node(ix + 1, iy + 1)                 # across edge (a1,a2)
        a4 = grid_node(ix - 1, iy + 1) if ix > 0 else None        # across edge (a2,a0)
        a5 = grid_node(ix + 1, iy - 1) if iy > 0 else None        # across edge (a0,a1)
        ele_a = fea.ChElementShellBST()
        ele_a.SetNodes(a0, a1, a2, a3, a4, a5)
        ele_a.AddLayer(thickness, 0.0 * chrono.CH_DEG_TO_RAD, material)
        mesh.AddElement(ele_a)
        elements.append(ele_a)

        # Element B: main triangle (ix+1,iy+1), (ix,iy+1), (ix+1,iy)
        b0 = grid_node(ix + 1, iy + 1)
        b1 = grid_node(ix, iy + 1)
        b2 = grid_node(ix + 1, iy)
        b3 = grid_node(ix, iy)                          # across edge (b1,b2)
        b4 = grid_node(ix + 2, iy) if ix < n_div - 1 else None    # across edge (b2,b0)
        b5 = grid_node(ix, iy + 2) if iy < n_div - 1 else None    # across edge (b0,b1)
        ele_b = fea.ChElementShellBST()
        ele_b.SetNodes(b0, b1, b2, b3, b4, b5)
        ele_b.AddLayer(thickness, 0.0 * chrono.CH_DEG_TO_RAD, material)
        mesh.AddElement(ele_b)
        elements.append(ele_b)
keepalive["elements"] = elements

# Pin two diagonally-opposite corners so the unsupported span folds under gravity.
grid_node(0, 0).SetFixed(True)
grid_node(n_nodes_side - 1, n_nodes_side - 1).SetFixed(True)

sys.Add(mesh)

# === FEA visualization === displacement-colored shell + wireframe reference overlay
vis_shell = chrono.ChVisualShapeFEA()
vis_shell.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_DISP_NORM)
vis_shell.SetColormapRange(chrono.ChVector2d(0.0, start_height))
vis_shell.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(vis_shell)

vis_wire = chrono.ChVisualShapeFEA()
vis_wire.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_wire.SetWireframe(True)
mesh.AddVisualShapeFEA(vis_wire)
keepalive["vis_shell"] = vis_shell
keepalive["vis_wire"] = vis_wire

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Tablecloth folding - Kirchhoff shell FEA")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.6, -1.8, 1.2), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddGrid(0.1, 0.1, 20, 20, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === render-cadence outer loop, physics in an inner batch
center_node = grid_node(n_div // 2, n_div // 2)  # cache: tracked once, sampled per step


try:

    frame = 0
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid shell state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
