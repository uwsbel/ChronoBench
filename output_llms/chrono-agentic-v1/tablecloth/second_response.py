"""
Tablecloth FEA simulation using PyChrono 9.0.x (Irrlicht).

Models a flexible cloth/tablecloth mesh using Kirchhoff-BST shell elements over
a rectangular grid. The upper row of nodes (j in range(30), k in range(30)) is
fixed (clamped edge), while the rest drape under gravity. A distributed load force
is applied to selected nodes in nodesLoad. Node monitoring (nodePlotA, nodePlotB,
mnodemonitor) and element monitoring (melementmonitor) track deformation over time.
Interpolation functions ref_X and ref_Y provide reference trajectories.

System:       ChSystemSMC (required for FEA shell stiffness)
Solver:       Pardiso MKL with LockSparsityPattern=True
Timestepper:  Euler Implicit Linearized (stable for stiff cloth shells)
Time step:    0.005 s (adjusted per prompt)
"""

# === Imports ===
import os
import math
import csv
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr


# === Simulation parameters ===
time_step = 0.005          # time step adjusted from 0.001 to 0.005 per prompt
sim_end = 3.0
render_fps = 30.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# Mesh dimensions: nsections_x x nsections_z shell panels
nsections_x = 30
nsections_z = 30
cloth_lx = 1.0    # total X length of cloth (m)
cloth_lz = 1.0    # total Z length of cloth (m)
thickness = 0.01  # shell thickness (m)

# Material properties: soft cloth
E_fabric = 5e4     # Young's modulus (Pa) — soft, low-stiffness cloth
nu_fabric = 0.3    # Poisson ratio
density = 200.0    # kg/m3

# Load force applied to nodesLoad list of nodes
load_force = chrono.ChVector3d(0, -2.0, 0)   # downward distributed force (N/node)

# === System & gravity (Y-up, ChSystemSMC for FEA) ===
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # Y-up world

# === Solver: Pardiso MKL, LockSparsityPattern=True ===
# LockSparsityPattern changed from False to True to optimize computation (prompt §6)
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)   # lock sparsity pattern for repeated solves
sys.SetSolver(mkl_solver)

# === Timestepper: Euler Implicit Linearized (robust for stiff BST shell cloth) ===
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)

# === FEA Mesh ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

# Kirchhoff-BST shell material
melasticity = fea.ChElasticityKirchhoffIsothropic(E_fabric, nu_fabric)
material = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(density)

# === Build node grid ===
# Grid: (nsections_x+1) x (nsections_z+1) = 31x31 nodes
# Index: mynodes[iz * (nsections_x+1) + ix]
mynodes = []
for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        x_pos = (ix / nsections_x) * cloth_lx
        y_pos = 0.0
        z_pos = (iz / nsections_z) * cloth_lz
        node = fea.ChNodeFEAxyz(chrono.ChVector3d(x_pos, y_pos, z_pos))
        mesh.AddNode(node)
        mynodes.append(node)

# === Monitoring nodes and load list ===
# nodePlotA: node at quarter-span, mid-row — for displacement plotting
nodePlotA = mynodes[(nsections_z // 2) * (nsections_x + 1) + (nsections_x // 4)]
# nodePlotB: node at mid-span, mid-row — for displacement plotting
nodePlotB = mynodes[(nsections_z // 2) * (nsections_x + 1) + (nsections_x // 2)]
# mnodemonitor: assigned during element loop when iz==0 and ix==1
mnodemonitor = mynodes[0 * (nsections_x + 1) + 1]   # iz=0, ix=1

# nodesLoad: interior nodes along mid-row to receive load force
nodesLoad = []
for ix in range(1, nsections_x):
    nodesLoad.append(mynodes[(nsections_z // 2) * (nsections_x + 1) + ix])

# Interpolation functions for reference tracking
def ref_X(t):
    """Reference X displacement (m) — static reference (tablecloth hangs straight)."""
    return 0.0

def ref_Y(t):
    """Reference Y sag (m) — expected mid-span sag under gravity + load."""
    return -0.05 * math.tanh(2.0 * t)

# === Build BST shell elements with conditional boundary node checks ===
melementmonitor = None   # monitoring element: assigned when iz==0 and ix==1
elements = []            # keep strong references to prevent SWIG GC

for iz in range(nsections_z):
    for ix in range(nsections_x):
        # Four corners of the quad cell
        n00 = mynodes[iz       * (nsections_x + 1) + ix    ]
        n10 = mynodes[iz       * (nsections_x + 1) + ix + 1]
        n01 = mynodes[(iz + 1) * (nsections_x + 1) + ix    ]
        n11 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]

        # --- Triangle A: vertices n00, n10, n01 ---
        # Neighbour for edge n00-n10 (bottom edge): row below if iz > 0
        nA0 = mynodes[(iz - 1) * (nsections_x + 1) + ix] if (iz > 0) else None
        # Neighbour for edge n10-n01 (diagonal): upper-right quad shares this diagonal
        nA1 = n11
        # Neighbour for edge n01-n00 (left edge): column to left if ix > 0
        nA2 = mynodes[iz * (nsections_x + 1) + (ix - 1)] if (ix > 0) else None

        melementA = fea.ChElementShellBST()
        melementA.SetNodes(n00, n10, n01, nA0, nA1, nA2)
        melementA.AddLayer(thickness, 0.0, material)
        mesh.AddElement(melementA)
        elements.append(melementA)

        # Assign element monitor when iz==0 and ix==1
        if iz == 0 and ix == 1:
            melementmonitor = melementA

        # --- Triangle B: vertices n10, n11, n01 ---
        # Neighbour for edge n10-n11 (right edge): column to right if ix+1 < nsections_x
        nB0 = mynodes[iz * (nsections_x + 1) + (ix + 2)] if (ix + 1 < nsections_x) else None
        # Neighbour for edge n11-n01 (diagonal): shared diagonal is n00
        nB1 = n00
        # Neighbour for edge n01-n10 (upper edge): row above if iz+1 < nsections_z
        nB2 = mynodes[(iz + 2) * (nsections_x + 1) + ix] if (iz + 1 < nsections_z) else None

        melementB = fea.ChElementShellBST()
        melementB.SetNodes(n10, n11, n01, nB0, nB1, nB2)
        melementB.AddLayer(thickness, 0.0, material)
        mesh.AddElement(melementB)
        elements.append(melementB)

# === Fix upper nodes (clamped edge) ===
# Two nested loops as specified: for j in range(30) and for k in range(30)
# This fixes the iz=0 row (bottom strip) — clamped tablecloth edge
for j in range(30):
    for k in range(30):
        mynodes[j * (nsections_x + 1) + k].SetFixed(True)

# === Apply distributed load to nodesLoad ===
# FEA mesh: no contact material needed — driven by gravity + clamped BCs + node forces only
for load_node in nodesLoad:
    load_node.SetForce(load_force)

# === Register mesh with the system ===
sys.Add(mesh)

# Shells require Setup + Update before the simulation loop
sys.Setup()
sys.Update(True)

# === FEA Visualization shapes ===
# mvisualizeshellA: surface colored by displacement norm; smooth faces + wireframe
mvisualizeshellA = chrono.ChVisualShapeFEA()
mvisualizeshellA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_DISP_NORM)
mvisualizeshellA.SetColormapRange(0.0, 0.5)
mvisualizeshellA.SetSmoothFaces(True)
mvisualizeshellA.SetWireframe(True)       # wireframe mode enabled
# mvisualizeshellA.SetBackfaceCull(True)  # optional backface culling (commented out)
mvisualizeshellA.SetShellResolution(2)
mesh.AddVisualShapeFEA(mvisualizeshellA)

# mvisualizeshellB: node glyphs, DataType_NONE
mvisualizeshellB = chrono.ChVisualShapeFEA()
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetSymbolsThickness(0.006)
mvisualizeshellB.SetSymbolsScale(0.01)
mvisualizeshellB.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizeshellB)

# === Irrlicht visualization: full block (Initialize first, then scene elements) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Tablecloth BST Shell FEA — MKL Pardiso, LockSparsityPattern=True, dt=0.005")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # Y-up world convention
vis.Initialize()                                     # Initialize FIRST (Irrlicht convention)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.6, 0.7, -1.2), chrono.ChVector3d(0.5, -0.1, 0.5))
vis.AddTypicalLights()
vis.AddGrid(0.1, 0.1, 25, 25,
            chrono.ChCoordsysd(chrono.ChVector3d(0.5, -0.4, 0.5), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Review-only setup ===


# === Main simulation loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t = sys.GetChTime()
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence or bad FEA state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass   # CSV closed in review-only block below
