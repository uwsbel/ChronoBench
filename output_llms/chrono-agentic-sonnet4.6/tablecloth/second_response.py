"""
Tablecloth FEA simulation using PyChrono 9.0.x with Irrlicht visualization.

Models a 1m x 1m rectangular cloth as a 40x40 BST Kirchhoff shell mesh draping under gravity.
Uses ChSystemSMC + Pardiso MKL solver with LockSparsityPattern=True.
The cloth has its upper 30x30 nodes fixed. Selected nodes have a load force applied.
Element monitoring is assigned for the element at (iz==0, ix==1). Two FEA visual shapes
show surface deformation and node dot positions respectively. Time step is 0.005 s.

System: ChSystemSMC
FEA element: ChElementShellBST (Kirchhoff shell)
Solver: Pardiso MKL (LockSparsityPattern=True)
Timestepper: HHT (SetStepControl=False, SetAlpha=-0.2 for numerical dissipation)
Expected behavior: cloth droops downward under gravity from fixed upper-left corner region.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

# === Simulation parameters ===
time_step = 0.005           # s — per input specification (changed from 0.001)
sim_end = 2.0               # s
render_fps = 30.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# Cloth geometry
L_x = 1.0                   # m, cloth length in X
L_z = 1.0                   # m, cloth length in Z
nsections_x = 40            # mesh cells along X
nsections_z = 40            # mesh cells along Z

# Material properties (from reference BST shell tablecloth demo)
density = 100.0             # kg/m^3
E = 6e4                     # Pa
nu = 0.0                    # Poisson ratio
thickness = 0.01            # m

# Load force applied to selected nodes (input spec item 1)
load_force = chrono.ChVector3d(0, 0, 0)  # N — zero by default

# === System & gravity (Y-up for FEA family) ===
sys = chrono.ChSystemSMC()
# Default gravity (0, -9.81, 0) applies automatically

# === FEA mesh construction (BST shell tablecloth) ===
mesh = fea.ChMesh()
# mesh.SetAutomaticGravity(False)  # uncomment to disable body-gravity on the mesh

# Register mesh with system
sys.Add(mesh)

# Node monitoring and loading variable declarations (input spec item 1)
nodePlotA = None
nodePlotB = None
nodesLoad = []              # nodes for load application

mnodemonitor = None
melementmonitor = None

# Shell material: isotropic Kirchhoff elasticity (API spelling: Isothropic)
melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)
material = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(density)

# === Build grid of FEA nodes (Y-up world, cloth horizontal at y=0 in XZ plane) ===
mynodes = []
for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        p = chrono.ChVector3d(ix * (L_x / nsections_x), 0, iz * (L_z / nsections_z))
        mnode = fea.ChNodeFEAxyz(p)
        mesh.AddNode(mnode)
        mynodes.append(mnode)

# Assign nodePlotA and nodePlotB (input spec item 1)
nodePlotA = mynodes[0]              # (ix=0, iz=0) — will be fixed
nodePlotB = mynodes[nsections_x]    # (ix=40, iz=0) — free edge node

# Build nodesLoad list (input spec item 1)
for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        nodesLoad.append(mynodes[iz * (nsections_x + 1) + ix])

# === Build BST shell elements over the grid ===
for iz in range(nsections_z):
    for ix in range(nsections_x):
        # --- Triangle A: lower-left, lower-right, upper-left ---
        melementA = fea.ChElementShellBST()
        mesh.AddElement(melementA)

        # Element monitoring assignment (input spec item 3): iz==0 and ix==1
        if iz == 0 and ix == 1:
            melementmonitor = melementA

        # Boundary stencil with conditional checks (input spec item 2: ix>0, iz>0)
        boundary_1 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]
        boundary_2 = mynodes[(iz + 1) * (nsections_x + 1) + ix - 1] if (ix > 0) else None
        boundary_3 = mynodes[(iz - 1) * (nsections_x + 1) + ix + 1] if (iz > 0) else None

        melementA.SetNodes(
            mynodes[(iz    ) * (nsections_x + 1) + ix    ],
            mynodes[(iz    ) * (nsections_x + 1) + ix + 1],
            mynodes[(iz + 1) * (nsections_x + 1) + ix    ],
            boundary_1, boundary_2, boundary_3)
        melementA.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)

        # --- Triangle B: upper-right, upper-left, lower-right ---
        melementB = fea.ChElementShellBST()
        mesh.AddElement(melementB)

        boundary_1 = mynodes[(iz    ) * (nsections_x + 1) + ix    ]
        boundary_2 = mynodes[(iz    ) * (nsections_x + 1) + ix + 2] if (ix < nsections_x - 1) else None
        boundary_3 = mynodes[(iz + 2) * (nsections_x + 1) + ix    ] if (iz < nsections_z - 1) else None

        melementB.SetNodes(
            mynodes[(iz + 1) * (nsections_x + 1) + ix + 1],
            mynodes[(iz + 1) * (nsections_x + 1) + ix    ],
            mynodes[(iz    ) * (nsections_x + 1) + ix + 1],
            boundary_1, boundary_2, boundary_3)
        melementB.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)

# === Fix upper nodes (input spec item 4): nested loops j in range(30), k in range(30) ===
for j in range(30):
    for k in range(30):
        mynodes[j * (nsections_x + 1) + k].SetFixed(True)

# Monitoring node: a free interior node for position tracking
mnodemonitor = mynodes[35 * (nsections_x + 1) + 35]  # free interior at iz=35, ix=35

# === Solver: Pardiso MKL with locked sparsity pattern (input spec item 6) ===
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)   # locked pattern optimizes repeated factorizations
sys.SetSolver(mkl_solver)

# === Timestepper: HHT with numerical dissipation for stability ===
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
ts.SetAlpha(-0.2)           # negative alpha adds HHT numerical dissipation
sys.SetTimestepper(ts)

# sys.Setup() and sys.Update() required for shell elements before the loop
sys.Setup()
sys.Update()

# === FEA Visualization shapes (two-shape canonical pattern) ===
# Shape A: surface display with smooth faces (input spec item 5)
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetSmoothFaces(True)
mvisualizeshellA.SetWireframe(False)
mvisualizeshellA.SetShellResolution(2)
# mvisualizeshellA.SetBackfaceCull(True)   # optional backface culling — commented out per spec
mesh.AddVisualShapeFEA(mvisualizeshellA)

# Shape B: node glyph markers with DataType_NONE (input spec item 5)
mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetSymbolsThickness(0.006)
mesh.AddVisualShapeFEA(mvisualizeshellB)

# === Visualization: full Irrlicht block (Initialize first, scene elements after) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Shells FEA — tablecloth BST elements")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 0.3, 1.3), chrono.ChVector3d(0.5, -0.3, 0.5))
vis.AddTypicalLights()

# === Review-only setup ===

# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
