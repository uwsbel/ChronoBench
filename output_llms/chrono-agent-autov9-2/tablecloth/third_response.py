"""Draping tablecloth — thin Kirchhoff-Love shell modeled with triangular BST elements.

Model:
    A square fabric sheet (the "tablecloth") discretized as a regular triangular
    mesh of `ChElementShellBST` elements over `ChNodeFEAxyz` nodes, with a single
    isotropic `ChMaterialShellKirchhoff` layer (membrane + edge-stencil bending).
    The four corner nodes are pinned (held fixed); the rest of the sheet sags and
    drapes under gravity.

System:
    `ChSystemSMC` (FEA requires a smooth-contact system) with a PardisoMKL direct
    solver and the EULER_IMPLICIT_LINEARIZED timestepper, which is stable for the
    stiff membrane terms of a thin shell.

Bodies / mesh:
    One `ChMesh` holding the node grid and the BST shell elements. No rigid bodies
    and no contact are present — the cloth is driven purely by gravity and the
    corner pin constraints — so no collision system or contact material is set up.

Expected behavior:
    Starting flat and horizontal, the interior of the sheet falls and forms a
    smooth catenary-like sag between the four pinned corners, settling into a
    static drape. The triangular shell surface is rendered with a `ChVisualShapeFEA`
    showing smooth faces, a wireframe overlay, and backface culling enabled.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

# === Named constants === geometry / material / timing (no bare literals downstream)
cloth_size = 1.0           # m, edge length of the square sheet
n_div = 12                 # elements per edge -> (n_div+1)^2 nodes
node_per_edge = n_div + 1
cell = cloth_size / n_div  # spacing between adjacent nodes
z0 = 1.0                   # initial height of the flat sheet above ground

thickness = 0.002          # m, fabric thickness
density = 200.0            # kg/m^3, light fabric
E = 1.0e6                  # Pa, low Young's modulus -> drapes readily
nu = 0.3                   # Poisson ratio

time_step = 1.0e-3
sim_end = 3.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))           # precomputed once
gravity = chrono.ChVector3d(0, 0, -9.81)

# === System & gravity === SMC + direct MKL solver, linearized implicit stepper
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(gravity)
mkl_solver = mkl.ChSolverPardisoMKL()        # direct solver required for FEA stiffness
mkl_solver.LockSparsityPattern(True)         # mesh connectivity is fixed -> reuse pattern
sys.SetSolver(mkl_solver)
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)
# Pure FEA shell driven by gravity + corner pins: no contact, so no collision
# system and no contact material are configured.

# === FEA mesh: nodes ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)   # gravity applied through the element mass

# Build the (node_per_edge x node_per_edge) grid of position nodes.
nodes = []                       # cache: node grid kept referenced (avoid SWIG GC)
for iy in range(node_per_edge):
    row = []
    for ix in range(node_per_edge):
        px = ix * cell - 0.5 * cloth_size
        py = iy * cell - 0.5 * cloth_size
        nd = fea.ChNodeFEAxyz(chrono.ChVector3d(px, py, z0))
        # nodal mass is supplied by the BST element (density x thickness x area)
        mesh.AddNode(nd)
        row.append(nd)
    nodes.append(row)

# Pin the four corners so the sheet drapes between them.
corner_nodes = [
    nodes[0][0],
    nodes[0][node_per_edge - 1],
    nodes[node_per_edge - 1][0],
    nodes[node_per_edge - 1][node_per_edge - 1],
]
for cn in corner_nodes:
    cn.SetFixed(True)
assert all(cn.IsFixed() for cn in corner_nodes), "corner pins not applied"

# === FEA material: isotropic Kirchhoff shell ===
elasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)
material = fea.ChMaterialShellKirchhoff(elasticity)
material.SetDensity(density)


def node_at(ix, iy):
    """Return the grid node at (ix, iy), or None if out of range (boundary stencil)."""
    if 0 <= ix < node_per_edge and 0 <= iy < node_per_edge:
        return nodes[iy][ix]
    return None


# === FEA elements: triangular BST shells ===
# Each grid cell is split into two triangles. A BST element carries its three
# triangle nodes (n0,n1,n2) plus up to three "opposite" neighbour nodes (n3,n4,n5)
# across each edge, which provide the discrete bending (curvature) stencil. A
# neighbour that does not exist (mesh boundary) is passed as None.
elements = []                    # cache: elements kept referenced (avoid SWIG GC)
for iy in range(n_div):
    for ix in range(n_div):
        # Lower-left triangle: main nodes (ix,iy) (ix+1,iy) (ix,iy+1).
        # b1 across the hypotenuse, b2 across the left edge, b3 across the
        # bottom edge — None where the edge lies on the mesh boundary.
        b1 = node_at(ix + 1, iy + 1)
        b2 = node_at(ix - 1, iy + 1) if ix > 0 else None
        b3 = node_at(ix + 1, iy - 1) if iy > 0 else None
        el_a = fea.ChElementShellBST()
        el_a.SetNodes(
            node_at(ix, iy), node_at(ix + 1, iy), node_at(ix, iy + 1),
            b1, b2, b3,
        )
        el_a.AddLayer(thickness, 0.0, material)
        mesh.AddElement(el_a)
        elements.append(el_a)

        # Upper-right triangle: main nodes (ix+1,iy+1) (ix,iy+1) (ix+1,iy).
        b1 = node_at(ix, iy)
        b2 = node_at(ix + 2, iy) if ix < n_div - 1 else None
        b3 = node_at(ix, iy + 2) if iy < n_div - 1 else None
        el_b = fea.ChElementShellBST()
        el_b.SetNodes(
            node_at(ix + 1, iy + 1), node_at(ix, iy + 1), node_at(ix + 1, iy),
            b1, b2, b3,
        )
        el_b.AddLayer(thickness, 0.0, material)
        mesh.AddElement(el_b)
        elements.append(el_b)

sys.Add(mesh)
sys.Setup()   # build the sparse system once the full mesh topology is registered

# === Mesh visualization: smooth + wireframe overlay + backface culling ===
mvisualizeshellA = chrono.ChVisualShapeFEA()
mvisualizeshellA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
mvisualizeshellA.SetColormapRange(chrono.ChVector2d(0.0, 1.0))
mvisualizeshellA.SetShellResolution(2)
mvisualizeshellA.SetSmoothFaces(True)
mvisualizeshellA.SetWireframe(True)
mvisualizeshellA.SetBackfaceCull(True)
mesh.AddVisualShapeFEA(mvisualizeshellA)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Draping tablecloth — BST Kirchhoff shell")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.6, -1.8, 1.6), chrono.ChVector3d(0, 0, 0.6))
vis.AddTypicalLights()
vis.AddGrid(0.25, 0.25, 16, 16,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === render-cadence outer loop, physics in inner batch
center = nodes[node_per_edge // 2][node_per_edge // 2]   # cache: sag-tracking node

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
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid FEA state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
