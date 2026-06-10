"""
Tablecloth FEA simulation using Kirchhoff BST shell elements.

Models a 1x1 m cloth mesh (40x40 sections) with isotropic Kirchhoff elasticity,
using ChSystemSMC + PardisoMKL solver. The upper-left 30x30 nodes are fixed to
simulate the tablecloth pinned at a corner region. Gravity acts on the free portion,
causing the cloth to drape dynamically.

Visualization uses two ChVisualShapeFEA shapes on the mesh:
  - mvisualizeshellA: surface render with smooth faces, wireframe mode, and backface culling
  - mvisualizeshellB: node-dot glyph overlay

System type: ChSystemSMC
Solver: PardisoMKL (locked sparsity pattern)
"""

import errno
import os

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl


# === Output directory ===
out_dir = chrono.GetChronoOutputPath() + "FEA_SHELLS_BST"

try:
    os.mkdir(out_dir)
except OSError as exc:
    if exc.errno != errno.EEXIST:
        print("Error creating output directory")

# === System & gravity ===
sys = chrono.ChSystemSMC()

# === FEA mesh: Kirchhoff BST shell cloth ===
mesh = fea.ChMesh()
sys.Add(mesh)
mesh.SetAutomaticGravity(True)

# Shell material: isotropic Kirchhoff elasticity
density = 100       # kg/m^3
E = 6e4             # Young's modulus (Pa) — compliant cloth
nu = 0.0            # Poisson's ratio
thickness = 0.01    # shell thickness (m)

melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)
material = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(density)

# Mesh grid dimensions
L_x = 1
nsections_x = 40
L_z = 1
nsections_z = 40

# Monitoring/plotting handles
nodePlotA = fea.ChNodeFEAxyz()
nodePlotB = fea.ChNodeFEAxyz()
nodesLoad = []

ref_X = chrono.ChFunctionInterp()
ref_Y = chrono.ChFunctionInterp()
load_force = chrono.ChVector3d()

mnodemonitor = fea.ChNodeFEAxyz()
melementmonitor = fea.ChElementShellBST()

# Build node grid
mynodes = []
for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        p = chrono.ChVector3d(ix * (L_x / nsections_x), 0, iz * (L_z / nsections_z))
        mnode = fea.ChNodeFEAxyz(p)
        mesh.AddNode(mnode)
        mynodes.append(mnode)

# Build BST shell elements (two triangles per quad cell)
for iz in range(nsections_z):
    for ix in range(nsections_x):
        # Triangle A
        melementA = fea.ChElementShellBST()
        mesh.AddElement(melementA)

        if iz == 0 and ix == 1:
            melementmonitor = melementA

        boundary_1 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]
        boundary_2 = mynodes[(iz + 1) * (nsections_x + 1) + ix - 1] if ix > 0 else None
        boundary_3 = mynodes[(iz - 1) * (nsections_x + 1) + ix + 1] if iz > 0 else None

        melementA.SetNodes(
            mynodes[iz * (nsections_x + 1) + ix],
            mynodes[iz * (nsections_x + 1) + ix + 1],
            mynodes[(iz + 1) * (nsections_x + 1) + ix],
            boundary_1, boundary_2, boundary_3
        )
        melementA.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)

        # Triangle B
        melementB = fea.ChElementShellBST()
        mesh.AddElement(melementB)

        boundary_1 = mynodes[iz * (nsections_x + 1) + ix]
        boundary_2 = mynodes[iz * (nsections_x + 1) + ix + 2] if ix < nsections_x - 1 else None
        boundary_3 = mynodes[(iz + 2) * (nsections_x + 1) + ix] if iz < nsections_z - 1 else None

        melementB.SetNodes(
            mynodes[(iz + 1) * (nsections_x + 1) + ix + 1],
            mynodes[(iz + 1) * (nsections_x + 1) + ix],
            mynodes[iz * (nsections_x + 1) + ix + 1],
            boundary_1, boundary_2, boundary_3
        )
        melementB.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)

# Fix upper-left 30x30 corner nodes (pinned tablecloth edge)
for j in range(30):
    for k in range(30):
        mynodes[j * (nsections_x + 1) + k].SetFixed(True)

# === FEA visualization shapes ===
# Shape A: surface render with smooth faces, wireframe mode, and backface culling enabled
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetSmoothFaces(True)
mvisualizeshellA.SetWireframe(True)
mvisualizeshellA.SetShellResolution(2)
mvisualizeshellA.SetBackfaceCull(True)
mesh.AddVisualShapeFEA(mvisualizeshellA)

# Shape B: node-dot glyph overlay (DataType_NONE + NODE_DOT_POS glyphs)
mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetSymbolsThickness(0.006)
mesh.AddVisualShapeFEA(mvisualizeshellB)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Shells FEA test: triangle BST elements")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 0.3, 1.3), chrono.ChVector3d(0.5, -0.3, 0.5))
vis.AddTypicalLights()

# === Solver: PardisoMKL with locked sparsity ===
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)
sys.SetSolver(mkl_solver)

# === Simulation parameters ===
timestep = 0.005    # s — stable timestep for this shell stiffness
sim_end = 2.0       # s
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * timestep)))  # precomputed once

# Pre-simulation setup required for FEA shells
sys.Setup()
sys.Update()

# Interpolation records (post-processing references)
rec_X = chrono.ChFunctionInterp()
rec_Y = chrono.ChFunctionInterp()


frame = 0
# === Main loop ===
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(timestep)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:
    import traceback
    traceback.print_exc()
    raise
