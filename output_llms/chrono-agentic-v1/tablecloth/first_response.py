"""
Tablecloth Folding Simulation — PyChrono 9.0.x (Irrlicht, Y-up)

Models a flexible tablecloth using Kirchhoff BST (Batoz-Sabir-Tabarrok) shell
elements with an isotropic Kirchhoff material. A rectangular grid of triangular
BST elements is built node-by-node over a planar mesh. A block of corner nodes
is fixed to ground, and gravity pulls the free portion of the cloth downward,
producing realistic draping and folding behaviour.

System:     ChSystemSMC (required for FEA shell elements)
Solver:     PardisoMKL with locked sparsity pattern (required for stiff shells)
Timestepper: default (symplectic); no HHT in this build — direct MKL solver
             handles the stiffness
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
CLOTH_LX     = 1.0    # Width in X (m)
CLOTH_LZ     = 1.0    # Width in Z (m)
N_SECTIONS_X = 40     # Number of divisions in X (match official demo)
N_SECTIONS_Z = 40     # Number of divisions in Z

# Material properties (isotropic Kirchhoff) — match PyChrono BST reference values
CLOTH_DENSITY   = 100.0   # kg/m³ (light fabric-like material)
CLOTH_E         = 6e4     # Young's modulus (Pa) — soft fabric
CLOTH_NU        = 0.0     # Poisson ratio
CLOTH_THICKNESS = 0.01    # m

# Fixed boundary: a block of (N_FIXED_X × N_FIXED_Z) nodes at the "top-left"
N_FIXED_X = 30
N_FIXED_Z = 30

# Simulation parameters
TIME_STEP    = 0.005    # s — match official demo timestep
SIM_END      = 3.0      # s — enough time to see the cloth drape
RENDER_FPS   = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# === System & gravity (Y-up FEA convention) ===
sys = chrono.ChSystemSMC()
# Y-up: gravity pulls nodes in -Y direction
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === FEA mesh ===
mesh = fea.ChMesh()
sys.Add(mesh)                     # add mesh to system early (nodes need the mesh)

# Gravity is applied automatically to all mesh nodes by default

# === Material — isotropic Kirchhoff elasticity ===
# ChElasticityKirchhoffIsothropic: note correct Chrono spelling "Isothropic"
melasticity = fea.ChElasticityKirchhoffIsothropic(CLOTH_E, CLOTH_NU)
material    = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(CLOTH_DENSITY)

# === Build 1-D node array (ix + iz * (N_SECTIONS_X+1) indexing) ===
# Nodes lie in the XZ plane at y=0; gravity will pull them in -Y
mynodes = []  # flat list: index = iz*(N_SECTIONS_X+1) + ix
for iz in range(N_SECTIONS_Z + 1):
    for ix in range(N_SECTIONS_X + 1):
        x = ix * (CLOTH_LX / N_SECTIONS_X)
        y = 0.0
        z = iz * (CLOTH_LZ / N_SECTIONS_Z)
        node = fea.ChNodeFEAxyz(chrono.ChVector3d(x, y, z))
        mesh.AddNode(node)
        mynodes.append(node)

# Fix the boundary block (the "tablecloth edge" resting on the table rim)
for j in range(N_FIXED_Z):
    for k in range(N_FIXED_X):
        mynodes[j * (N_SECTIONS_X + 1) + k].SetFixed(True)

# === Build BST shell elements ===
# Each grid cell is split into 2 triangles (A and B) with correct BST neighbours.
for iz in range(N_SECTIONS_Z):
    for ix in range(N_SECTIONS_X):

        # --- Triangle A: nodes (iz,ix), (iz,ix+1), (iz+1,ix) ---
        # BST neighbours for each edge (opposite vertex across shared edge):
        # boundary_1 = node diagonally above-right (iz+1, ix+1) — always valid
        boundary_A1 = mynodes[(iz + 1) * (N_SECTIONS_X + 1) + ix + 1]
        # boundary_2 = node to the left of (iz+1,ix): (iz+1, ix-1), None at left edge
        boundary_A2 = mynodes[(iz + 1) * (N_SECTIONS_X + 1) + ix - 1] if ix > 0 else None
        # boundary_3 = node below (iz-1, ix+1), None at bottom edge
        boundary_A3 = mynodes[(iz - 1) * (N_SECTIONS_X + 1) + ix + 1] if iz > 0 else None

        eleA = fea.ChElementShellBST()
        mesh.AddElement(eleA)
        eleA.SetNodes(
            mynodes[(iz    ) * (N_SECTIONS_X + 1) + ix    ],
            mynodes[(iz    ) * (N_SECTIONS_X + 1) + ix + 1],
            mynodes[(iz + 1) * (N_SECTIONS_X + 1) + ix    ],
            boundary_A1, boundary_A2, boundary_A3,
        )
        eleA.AddLayer(CLOTH_THICKNESS, 0.0 * chrono.CH_DEG_TO_RAD, material)

        # --- Triangle B: nodes (iz+1,ix+1), (iz+1,ix), (iz,ix+1) ---
        # boundary_1 = node at (iz,ix) — always valid
        boundary_B1 = mynodes[(iz    ) * (N_SECTIONS_X + 1) + ix    ]
        # boundary_2 = node at (iz, ix+2), None at right edge
        boundary_B2 = mynodes[(iz    ) * (N_SECTIONS_X + 1) + ix + 2] if ix < N_SECTIONS_X - 1 else None
        # boundary_3 = node at (iz+2, ix), None at top edge
        boundary_B3 = mynodes[(iz + 2) * (N_SECTIONS_X + 1) + ix    ] if iz < N_SECTIONS_Z - 1 else None

        eleB = fea.ChElementShellBST()
        mesh.AddElement(eleB)
        eleB.SetNodes(
            mynodes[(iz + 1) * (N_SECTIONS_X + 1) + ix + 1],
            mynodes[(iz + 1) * (N_SECTIONS_X + 1) + ix    ],
            mynodes[(iz    ) * (N_SECTIONS_X + 1) + ix + 1],
            boundary_B1, boundary_B2, boundary_B3,
        )
        eleB.AddLayer(CLOTH_THICKNESS, 0.0 * chrono.CH_DEG_TO_RAD, material)

# === Solver — PardisoMKL with locked sparsity pattern ===
# Required for stiff Kirchhoff shell stiffness matrices
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)
sys.SetSolver(mkl_solver)

# Setup and update before the loop (mandatory for shell meshes)
sys.Setup()
sys.Update()

# === FEA Visualization shapes ===
# ChVisualShapeFEA(mesh) — mesh is required ctor arg when pardisomkl is imported
vis_shell = chrono.ChVisualShapeFEA(mesh)
vis_shell.SetShellResolution(2)
vis_shell.SetSmoothFaces(True)
vis_shell.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_DISP_NORM)
vis_shell.SetColorscaleMinMax(0.0, 0.3)
mesh.AddVisualShapeFEA(vis_shell)

vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_glyph.SetSymbolsThickness(0.006)
mesh.AddVisualShapeFEA(vis_glyph)

# === Visualization — Irrlicht window ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Tablecloth Folding — Kirchhoff BST Shell FEA")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)  # Y-up for FEA scene
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.0, 0.3, 1.3), chrono.ChVector3d(0.5, -0.3, 0.5))
vis.AddTypicalLights()
vis.AddGrid(
    0.1, 0.1, 12, 12,
    chrono.ChCoordsysd(chrono.ChVector3d(0.5, -0.6, 0.5), chrono.QUNIT),
    chrono.ChColor(0.3, 0.3, 0.3),
)

# Cache a free node near the cloth center for physics monitoring
# Use a node in the free-hanging portion (below the fixed block)
free_node_idx = N_FIXED_Z * (N_SECTIONS_X + 1) + N_SECTIONS_X // 2  # cache: fetched once
monitor_node  = mynodes[free_node_idx]  # cache: reused every step
init_y        = monitor_node.GetPos().y  # cache: initial Y for displacement calc

# === Review-only: recording and CSV setup ===

frame = 0  # consecutive frame counter for review video

# === Main loop ===
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            t  = sys.GetChTime()    # cache: sim time this step
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / bad mesh state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
