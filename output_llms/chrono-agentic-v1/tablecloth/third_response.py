"""
Tablecloth FEA simulation using Kirchhoff BST shell elements (ChSystemSMC).

A rectangular cloth mesh is built from ChNodeFEAxyz nodes connected by
ChElementShellBST triangular shell elements.  The four corner nodes are
fixed to model a tablecloth draped over a frame.  Gravity causes the cloth
to sag downward.  The primary visualization shape (mvisualizeshellA) has
smooth faces, wireframe overlay, and backface culling enabled, as requested.

System   : ChSystemSMC (required for FEA shell elements)
Solver   : Pardiso MKL (direct, required for stiff Kirchhoff shells)
Timestepper : HHT (canonical-minimal form)
World    : Y-up  (FEA convention; gravity = (0,-9.81,0))
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# === Named constants ===
# Tablecloth geometry
CLOTH_SIZE_X = 1.0        # metres — cloth width
CLOTH_SIZE_Y = 1.0        # metres — cloth depth (along Z in Y-up world)
N_DIVX      = 12          # elements per row (must be ≥ 1)
N_DIVY      = 12          # elements per column

# Material
THICKNESS   = 0.01        # m
DENSITY     = 100.0       # kg/m³  (light fabric)
E_MOD       = 2e4         # Pa     (very flexible cloth)
NU          = 0.3         # Poisson ratio

# Simulation timing
TIME_STEP   = 1e-3        # s  (stiff shells → 1 ms)
SIM_END     = 5.0         # s
RENDER_FPS  = 50.0

# Precomputed
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# === System, gravity, solver ===
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))   # Y-up, gravity downward

# FEA shell with a pure jointed mesh — no rigid-body contact → no collision system needed.
# (All bodies are constrained mesh nodes fixed to ground; no collision shapes used.)

solver = mkl.ChSolverPardisoMKL()
sys.SetSolver(solver)

# Use Euler implicit linearized (default) for Kirchhoff BST shells — stable and fast
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)

# === FEA Mesh — Kirchhoff BST shell elements ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

# Isotropic Kirchhoff material
melasticity = fea.ChElasticityKirchhoffIsothropic(E_MOD, NU)
material    = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(DENSITY)

# Build a (N_DIVX+1) x (N_DIVY+1) grid of fea.ChNodeFEAxyz nodes
# in the XZ plane (Y is up, cloth hangs in X-Z, sags in -Y)
nodes = []
for iy in range(N_DIVY + 1):
    row = []
    for ix in range(N_DIVX + 1):
        x = (ix / N_DIVX) * CLOTH_SIZE_X - CLOTH_SIZE_X * 0.5
        z = (iy / N_DIVY) * CLOTH_SIZE_Y - CLOTH_SIZE_Y * 0.5
        node = fea.ChNodeFEAxyz(chrono.ChVector3d(x, 0.0, z))
        mesh.AddNode(node)
        row.append(node)
    nodes.append(row)

# Fix the four corner nodes to model a cloth pinned at corners
nodes[0][0].SetFixed(True)
nodes[0][N_DIVX].SetFixed(True)
nodes[N_DIVY][0].SetFixed(True)
nodes[N_DIVY][N_DIVX].SetFixed(True)

# Add ChElementShellBST elements (two triangles per quad cell)
# BST needs 6 nodes: 3 main corner nodes + 3 'neighbour' nodes (one opposite each edge)
# Neighbour is None at boundary edges.
def neighbour(nx, ny, rows, cols):
    """Return node at (nx,ny) or None if out of bounds."""
    if 0 <= nx <= rows and 0 <= ny <= cols:
        return nodes[nx][ny]
    return None

for iy in range(N_DIVY):
    for ix in range(N_DIVX):
        # Lower triangle: nodes at (iy,ix), (iy+1,ix), (iy,ix+1)
        n0 = nodes[iy    ][ix    ]
        n1 = nodes[iy + 1][ix    ]
        n2 = nodes[iy    ][ix + 1]
        # Neighbours opposite each edge
        b0 = neighbour(iy + 1, ix + 1, N_DIVY, N_DIVX)  # opposite edge n1-n2
        b1 = neighbour(iy - 1, ix    , N_DIVY, N_DIVX)  # opposite edge n0-n2
        b2 = neighbour(iy    , ix - 1, N_DIVY, N_DIVX)  # opposite edge n0-n1

        ele0 = fea.ChElementShellBST()
        ele0.SetNodes(n0, n1, n2, b0, b1, b2)
        ele0.AddLayer(THICKNESS, 0, material)
        mesh.AddElement(ele0)

        # Upper triangle: nodes at (iy+1,ix+1), (iy,ix+1), (iy+1,ix)
        n3 = nodes[iy + 1][ix + 1]
        n4 = nodes[iy    ][ix + 1]
        n5 = nodes[iy + 1][ix    ]
        c0 = neighbour(iy    , ix    , N_DIVY, N_DIVX)  # opposite edge n4-n5
        c1 = neighbour(iy + 2, ix + 1, N_DIVY, N_DIVX) # opposite edge n3-n5
        c2 = neighbour(iy + 1, ix + 2, N_DIVY, N_DIVX) # opposite edge n3-n4

        ele1 = fea.ChElementShellBST()
        ele1.SetNodes(n3, n4, n5, c0, c1, c2)
        ele1.AddLayer(THICKNESS, 0, material)
        mesh.AddElement(ele1)

sys.Add(mesh)

# === FEA Visualization (two-shape canonical pattern) ===
# In this 9.0.0 build ChVisualShapeFEA() takes NO constructor arguments.
# Shape 1 — surface/scalar field, named mvisualizeshellA (as referenced in prompt)
mvisualizeshellA = chrono.ChVisualShapeFEA()
mvisualizeshellA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
mvisualizeshellA.SetColormapRange(-0.5, 0.5)
mvisualizeshellA.SetSmoothFaces(True)       # smooth face shading (requested)
mvisualizeshellA.SetWireframe(True)         # wireframe overlay (requested)
mvisualizeshellA.SetBackfaceCull(True)      # backface culling (requested)
mvisualizeshellA.SetShellResolution(2)
mesh.AddVisualShapeFEA(mvisualizeshellA)

# Shape 2 — node glyphs
mvisualizeshellB = chrono.ChVisualShapeFEA()
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizeshellB.SetSymbolsThickness(0.006)
mvisualizeshellB.SetSymbolsScale(0.01)
mvisualizeshellB.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizeshellB)

# Setup must be called after building the mesh, before the loop
sys.Setup()
sys.Update(True)

# === Visualization — full Irrlicht scene ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Tablecloth FEA — BST Kirchhoff Shell (smooth + wireframe + backface cull)")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # Y-up for this FEA scene
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(0.0, 1.5, -2.0),   # camera eye — elevated, slightly behind
    chrono.ChVector3d(0.0, -0.2, 0.0),   # look-at cloth center
)
vis.AddTypicalLights()
vis.AddGrid(
    0.1, 0.1, 20, 20,
    chrono.ChCoordsysd(chrono.ChVector3d(0, -0.5, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

# === Review-only setup ===


# Cache frequently accessed objects
corner_node  = nodes[0][0]    # cache: corner node, used for logging
center_node  = nodes[N_DIVY // 2][N_DIVX // 2]  # cache: center node of cloth

# === Main simulation loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            t = sys.GetChTime()
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad mesh state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
