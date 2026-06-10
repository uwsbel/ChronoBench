"""Draping tablecloth modeled with Kirchhoff BST triangular shell elements (FEA).

System: ChSystemSMC (required for stiff Kirchhoff shells) with the PardisoMKL
direct solver. A square sheet is discretized into a grid of ChNodeFEAxyz nodes
and meshed with paired ChElementShellBST triangles (each carrying a thin layer of
an isotropic Kirchhoff material). One corner block of nodes is pinned (SetFixed),
so under gravity the unpinned portion of the cloth sags and drapes.

Visualization: a surface ChVisualShapeFEA (mvisualizeshellA) rendered with smooth
faces, wireframe overlay, and backface culling enabled, plus a node-dot glyph
shape, shown in an Irrlicht window. Expected behavior: the free part of the sheet
falls and folds under gravity while the fixed corner stays put.
"""

import errno
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl

# === Constants === geometry, material, and time-stepping parameters
DENSITY = 100        # kg/m^3 cloth density
E = 6e4              # Pa Young's modulus (soft cloth)
NU = 0.0             # Poisson ratio
THICKNESS = 0.01     # m shell thickness
L_X = 1              # m sheet extent along X
L_Z = 1              # m sheet extent along Z
NSECTIONS_X = 40     # element columns
NSECTIONS_Z = 40     # element rows
N_FIXED = 30         # pinned corner block is N_FIXED x N_FIXED nodes
TIME_STEP = 0.005    # s stiff-shell timestep
SIM_END = 5.0        # s recording horizon
RENDER_FPS = 50.0
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once


# === System & gravity === SMC system with a direct solver for stiff shells
sys = chrono.ChSystemSMC()

# Container mesh for the FEA nodes and shell elements.
mesh = fea.ChMesh()
sys.Add(mesh)

# === Material === isotropic Kirchhoff shell material wrapped over an elasticity law
melasticity = fea.ChElasticityKirchhoffIsothropic(E, NU)
material = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(DENSITY)

# === Nodes === planar grid of xyz nodes in the X-Z plane (Y is up)
mynodes = []
for iz in range(NSECTIONS_Z + 1):
    for ix in range(NSECTIONS_X + 1):
        p = chrono.ChVector3d(ix * (L_X / NSECTIONS_X), 0, iz * (L_Z / NSECTIONS_Z))
        mnode = fea.ChNodeFEAxyz(p)
        mesh.AddNode(mnode)
        mynodes.append(mnode)

# === Elements === two BST triangles per cell with their neighbour stencil
for iz in range(NSECTIONS_Z):
    for ix in range(NSECTIONS_X):
        melementA = fea.ChElementShellBST()
        mesh.AddElement(melementA)

        boundary_1 = mynodes[(iz + 1) * (NSECTIONS_X + 1) + ix + 1]
        boundary_2 = mynodes[(iz + 1) * (NSECTIONS_X + 1) + ix - 1] if ix > 0 else None
        boundary_3 = mynodes[(iz - 1) * (NSECTIONS_X + 1) + ix + 1] if iz > 0 else None

        melementA.SetNodes(
            mynodes[(iz) * (NSECTIONS_X + 1) + ix],
            mynodes[(iz) * (NSECTIONS_X + 1) + ix + 1],
            mynodes[(iz + 1) * (NSECTIONS_X + 1) + ix],
            boundary_1, boundary_2, boundary_3)
        melementA.AddLayer(THICKNESS, 0 * chrono.CH_DEG_TO_RAD, material)

        melementB = fea.ChElementShellBST()
        mesh.AddElement(melementB)

        boundary_1 = mynodes[(iz) * (NSECTIONS_X + 1) + ix]
        boundary_2 = mynodes[(iz) * (NSECTIONS_X + 1) + ix + 2] if ix < NSECTIONS_X - 1 else None
        boundary_3 = mynodes[(iz + 2) * (NSECTIONS_X + 1) + ix] if iz < NSECTIONS_Z - 1 else None

        melementB.SetNodes(
            mynodes[(iz + 1) * (NSECTIONS_X + 1) + ix + 1],
            mynodes[(iz + 1) * (NSECTIONS_X + 1) + ix],
            mynodes[(iz) * (NSECTIONS_X + 1) + ix + 1],
            boundary_1, boundary_2, boundary_3)
        melementB.AddLayer(THICKNESS, 0 * chrono.CH_DEG_TO_RAD, material)

# Pin a corner block of nodes so the rest of the sheet drapes under gravity.
for j in range(N_FIXED):
    for k in range(N_FIXED):
        mynodes[j * (NSECTIONS_X + 1) + k].SetFixed(True)

# === Visualization assets === surface shape (smooth + wireframe + backface cull) + node glyphs
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetSmoothFaces(True)
mvisualizeshellA.SetWireframe(True)
mvisualizeshellA.SetShellResolution(2)
mvisualizeshellA.SetBackfaceCull(True)
mesh.AddVisualShapeFEA(mvisualizeshellA)

mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetSymbolsThickness(0.006)
mesh.AddVisualShapeFEA(mvisualizeshellB)

# === Visualization === Irrlicht window: sky + camera + lights (Y-up FEA scene)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Shells FEA test: triangle BST elements')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, .3, 1.3), chrono.ChVector3d(.5, -.3, .5))
vis.AddTypicalLights()

# === Solver === PardisoMKL direct solver (iterative solvers diverge on shells)
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)
sys.SetSolver(mkl_solver)

# Shell meshes require an explicit Setup/Update before the dynamic loop.
sys.Setup()
sys.Update()


# === Main loop === advance the FEA dynamics and render the draping sheet
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:    # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
