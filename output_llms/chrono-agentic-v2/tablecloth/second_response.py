"""
Tablecloth FEA simulation — turn 2 refinements.

Models a deformable tablecloth using ChElementShellBST Kirchhoff shell elements
on a grid, fixed along two upper edges (j=0..29 rows, k=0..29 columns of the
node array). Node monitoring variables (nodePlotA, nodePlotB) and a load-bearing
list (nodesLoad) are defined for tracking and loading interior nodes.  An
interpolation monitor element (melementmonitor) is assigned when iz==0 and ix==1.
Boundary nodes are constructed with conditional checks (ix>0, iz>0) to avoid
out-of-bounds indexing.  The Pardiso MKL solver uses LockSparsityPattern=True to
optimize repeated factorization.  Time step is 0.005 s (updated from 0.001).
System: ChSystemSMC + Pardiso MKL.  Gravity: (0, -9.81, 0) — Y-up convention.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# === Simulation constants ===
time_step   = 0.005      # updated from 0.001
sim_end     = 2.0        # seconds to simulate
render_fps  = 50.0
# precomputed once
render_every = max(1, round(1.0 / (render_fps * time_step)))

# Cloth geometry
nsections_x = 30         # nodes in X: nsections_x + 1
nsections_z = 30         # nodes in Z: nsections_z + 1
cloth_size_x = 0.8       # metres
cloth_size_z = 0.8

# Shell material properties
E_cloth    = 0.2e4       # Young's modulus (soft fabric)
nu_cloth   = 0.3         # Poisson's ratio
rho_cloth  = 100.0       # density  kg/m³
thickness  = 0.01        # shell thickness

# Applied load on nodesLoad list
load_force = chrono.ChVector3d(0, -2, 0)   # downward pull (N)

# === System setup (SMC + Pardiso MKL for stiff shells) ===
# FEA shells require ChSystemSMC and a direct sparse solver
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)   # Solver adjustment: lock for re-use performance
sys.SetSolver(mkl_solver)

# No collision system call — pure FEA jointed scene with no rigid-body contact

# === FEA mesh — Kirchhoff BST shell cloth ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)   # let Chrono distribute gravity on FEA nodes

# Shell material
melasticity = fea.ChElasticityKirchhoffIsothropic(E_cloth, nu_cloth)
material    = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(rho_cloth)

# Build node grid (nsections_x+1) x (nsections_z+1)
mynodes = []
for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        x = ix * cloth_size_x / nsections_x
        y = 0.0
        z = iz * cloth_size_z / nsections_z
        node = fea.ChNodeFEAxyz(chrono.ChVector3d(x, y, z))
        mesh.AddNode(node)
        mynodes.append(node)

# Monitoring nodes and load list (Node Monitoring and Loading Setup)
# nodePlotA: interior node for displacement tracking
nodePlotA    = mynodes[int(nsections_x / 2) * (nsections_x + 1) + int(nsections_z / 2)]
nodePlotB    = mynodes[int(nsections_x / 4) * (nsections_x + 1) + int(nsections_z / 4)]
nodesLoad    = []          # nodes to which load_force will be applied
# assign load to a ring of inner nodes
mnodemonitor  = nodePlotA  # cache: monitoring node
melementmonitor = None     # will be set when iz==0 and ix==1

# Reference interpolation values (tracked in the loop or post-processing)
ref_X_vals = []
ref_Y_vals = []

# Build shell elements with BST stencil
for iz in range(nsections_z):
    for ix in range(nsections_x):
        # Node indices for the two triangles in each quad
        n0 = iz       * (nsections_x + 1) + ix
        n1 = iz       * (nsections_x + 1) + ix + 1
        n2 = (iz + 1) * (nsections_x + 1) + ix + 1
        n3 = (iz + 1) * (nsections_x + 1) + ix

        # Boundary neighbour indices with conditional checks (ix>0, iz>0)
        # Upper-left triangle: n0, n1, n3  — neighbours: n2 (interior), left, below
        melA = fea.ChElementShellBST()
        nb_A0 = mynodes[iz * (nsections_x + 1) + ix + 2]        if ix + 2 <= nsections_x  else None
        nb_A1 = mynodes[(iz + 1) * (nsections_x + 1) + ix - 1]  if (ix > 0)               else None  # (ix > 0)
        nb_A2 = mynodes[(iz - 1) * (nsections_x + 1) + ix]      if (iz > 0)               else None  # (iz > 0)
        melA.SetNodes(mynodes[n0], mynodes[n1], mynodes[n3], nb_A0, nb_A1, nb_A2)
        melA.AddLayer(thickness, 0.0, material)
        mesh.AddElement(melA)

        # Lower-right triangle: n1, n2, n3 — neighbours
        melB = fea.ChElementShellBST()
        nb_B0 = mynodes[(iz + 2) * (nsections_x + 1) + ix + 1]  if iz + 2 <= nsections_z  else None
        nb_B1 = mynodes[iz * (nsections_x + 1) + ix + 2]        if ix + 2 <= nsections_x  else None
        nb_B2 = mynodes[(iz + 1) * (nsections_x + 1) + ix - 1]  if (ix > 0)               else None  # (ix > 0)
        melB.SetNodes(mynodes[n1], mynodes[n2], mynodes[n3], nb_B0, nb_B1, nb_B2)
        melB.AddLayer(thickness, 0.0, material)
        mesh.AddElement(melB)

        # Element Monitoring — assign melementmonitor when iz==0 and ix==1
        if iz == 0 and ix == 1:
            melementmonitor = melA

# Populate nodesLoad with a sample of interior nodes
for iz in range(5, nsections_z - 5, 5):
    for ix in range(5, nsections_x - 5, 5):
        nodesLoad.append(mynodes[iz * (nsections_x + 1) + ix])

# Apply load_force to nodesLoad
for nd in nodesLoad:
    nd.SetForce(load_force)

# Fix Upper Nodes — two nested loops for j, k in range(30)
# (Pins the top-edge strip of 30×30 nodes so the cloth hangs from them)
for j in range(30):
    for k in range(30):
        mynodes[j * (nsections_x + 1) + k].SetFixed(True)

sys.Add(mesh)

# Required for shells: call Setup + Update before the loop
sys.Setup()
sys.Update()

# === FEA Visualization (attach to mesh BEFORE vis.Initialize) ===
# Shape A: surface displacement norm
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_DISP_NORM)
mvisualizeshellA.SetColorscaleMinMax(0.0, 0.08)
mvisualizeshellA.SetSmoothFaces(True)
mvisualizeshellA.SetWireframe(False)
# mvisualizeshellA.SetBackfaceCull(True)   # optional backface culling (commented out)
mesh.AddVisualShapeFEA(mvisualizeshellA)

# Shape B: glyph overlay (DataType_NONE)
mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetSymbolsThickness(0.004)
mvisualizeshellB.SetSymbolsScale(0.008)
mvisualizeshellB.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizeshellB)

# === Visualization — full Irrlicht scene ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Tablecloth FEA — Shell BST")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)  # Y-up to match gravity
vis.Initialize()                                    # FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.5, -0.3, 1.4), chrono.ChVector3d(0.4, 0.0, 0.4))
vis.AddTypicalLights()

# === Review-only setup ===


frame = 0  # consecutive frame counter

# === Main loop ===
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad simulation state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
