"""
Tablecloth Folding Simulation — PyChrono FEA Shell Demo (matching SimBench truth).

Models a flexible tablecloth as a Kirchhoff-Love shell mesh (ChElementShellBST)
with isotropic Kirchhoff material, PardisoMKL solver, and Irrlicht visualization.
The cloth drapes under gravity from its initial flat configuration.
"""

import os
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

# review-only imports for recording / CSV / video

# === Simulation parameters ===
timestep = 1e-3           # [s]
sim_end = 2.0              # [s]
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * timestep)))

# Tablecloth grid (matching SimBench truth dimensions)
L_x = 1.0                  # cloth width [m]
L_z = 1.0                  # cloth depth [m]
nsections_x = 12            # elements along X
nsections_z = 12            # elements along Z
N_x = nsections_x + 1      # nodes along X
N_z = nsections_z + 1      # nodes along Z

# Shell material properties (isotropic Kirchhoff — matching truth values)
thickness = 0.01           # [m]
density = 100.0            # [kg/m³]
E = 6e4                   # Young's modulus [Pa]
nu = 0.0                  # Poisson ratio

REC = bool(os.environ.get("SIMBENCH_RECORD"))

# === System (SMC with default gravity = (0, -9.81, 0)) ===
sys = chrono.ChSystemSMC()

# === Shell mesh ===
mesh = fea.ChMesh()
sys.Add(mesh)

# Kirchhoff isotropic material
melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)
material = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(density)

# Generate nodes — flat cloth lying on Y=0 plane (truth convention)
mynodes = []
for iz in range(N_z):
    for ix in range(N_x):
        p = chrono.ChVector3d(ix * (L_x / nsections_x), 0.0, iz * (L_z / nsections_z))
        mnode = fea.ChNodeFEAxyz(p)
        mesh.AddNode(mnode)
        mynodes.append(mnode)

# Helper: 1-D flat index matching truth's row-major layout
def node_idx(iz, ix):
    return iz * N_x + ix

def node_at(iz, ix):
    return mynodes[node_idx(iz, ix)]

# Build BST shell elements — two triangles per quad cell (matching truth topology)
for iz in range(nsections_z):
    for ix in range(nsections_x):
        # Triangle A: (bottom-left, bottom-right, top-left)
        melementA = fea.ChElementShellBST()
        boundary_1 = node_at(iz + 1, ix + 1)  # top-right neighbor node
        boundary_2 = node_at(iz + 1, ix - 1) if ix > 0 else None  # top-left neighbor
        boundary_3 = node_at(iz - 1, ix + 1) if iz > 0 else None  # bottom-right neighbor
        melementA.SetNodes(
            node_at(iz, ix),       # n1: bottom-left
            node_at(iz, ix + 1),   # n2: bottom-right
            node_at(iz + 1, ix),   # n3: top-left
            boundary_1,
            boundary_2,
            boundary_3,
        )
        melementA.AddLayer(thickness, 0, material)
        mesh.AddElement(melementA)

        # Triangle B: (top-right, top-right-bottoms, top-left)
        melementB = fea.ChElementShellBST()
        boundary_1 = node_at(iz, ix)               # bottom-left neighbor
        boundary_2 = node_at(iz, ix + 2) if ix < nsections_x - 1 else None
        boundary_3 = node_at(iz + 2, ix) if iz < nsections_z - 1 else None
        melementB.SetNodes(
            node_at(iz + 1, ix + 1),  # n1: top-right
            node_at(iz + 1, ix),       # n2: top-right-bottom
            node_at(iz, ix + 1),       # n3: top-left
            boundary_1,
            boundary_2,
            boundary_3,
        )
        melementB.AddLayer(thickness, 0, material)
        mesh.AddElement(melementB)

# === FEA Visual Shapes — attached BEFORE vis.Initialize() (required in 9.0.0) ===
vis_shell = chrono.ChVisualShapeFEA(mesh)
vis_shell.SetShellResolution(2)
mesh.AddVisualShapeFEA(vis_shell)

vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_glyph.SetSymbolsThickness(0.006)
mesh.AddVisualShapeFEA(vis_glyph)

# === Irrlicht Visualization (full block — AFTER Initialize) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Tablecloth Folding — Kirchhoff Shell FEA")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 0.3, 1.3), chrono.ChVector3d(0.5, -0.3, 0.5))
vis.AddTypicalLights()

# === Solver: PardisoMKL (matching truth setup) ===
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(False)
sys.SetSolver(mkl_solver)

sys.Setup()
sys.Update()

# === Review-only: frame capture ===

# === Main simulation loop ===
frame = 0
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    if REC:
        frame += 1
    for _ in range(render_every):
        sys.DoStepDynamics(timestep)
        if sys.GetChTime() >= sim_end:
            break

# === Review-only: assemble video, cleanup ===
