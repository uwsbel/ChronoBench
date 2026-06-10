"""
BST Shell Tablecloth — FEA simulation using ChElementShellBST.

A cloth/tablecloth modeled with Kirchhoff BST shell elements, fixed along
the upper boundary, free elsewhere. Uses ChSystemSMC with PardisoMKL solver
and an Irrlicht visualization window.

Changes from truth (input2.txt):
  - nodePlotA/nodePlotB monitoring nodes, nodesLoad list, ref_X/ref_Y
    interpolation functions, load_force vector, mnodemonitor/melementmonitor
  - Boundary node conditional checks (ix > 0) and (iz > 0)
  - ementmonitor = melementA when (iz == 0 and ix == 1)
  - Upper 30x30 nodes fixed via nested loops
  - mvisualizeshellA: smooth faces, wireframe mode, backface culling options
  - mvisualizeshellB: SetFEMdataType(DataType_NONE)
  - LockSparsityPattern: True (was False)
  - time_step: 0.005 (was 0.001)
"""

import errno
import os

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

import sim_recording as rec

# === Output directory ===
out_dir = chrono.GetChronoOutputPath() + "FEA_SHELLS_BST"
try:
    os.mkdir(out_dir)
except OSError as exc:
    if exc.errno != errno.EEXIST:
        print("Error creating output directory")

# === System & gravity ===
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)
sys.Add(mesh)

# === Node monitoring and loading setup ===
nodePlotA = fea.ChNodeFEAxyz()  # Node for plotting
nodePlotB = fea.ChNodeFEAxyz()  # Node for plotting
nodesLoad = []  # List for applying loads

# Interpolation functions for reference tracking
ref_X = chrono.ChFunctionInterp()
ref_Y = chrono.ChFunctionInterp()

# Load force vector
load_force = chrono.ChVector3d()

# Monitoring nodes and elements
mnodemonitor = fea.ChNodeFEAxyz()  # Monitoring node
melementmonitor = fea.ChElementShellBST()  # Monitoring element

# === Material properties ===
density = 100
E = 6e4
nu = 0.0
thickness = 0.01

melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)
material = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(density)

# === Mesh dimensions ===
L_x = 1
nsections_x = 40
L_z = 1
nsections_z = 40

# === Create nodes ===
mynodes = []
for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        p = chrono.ChVector3d(ix * (L_x / nsections_x), 0, iz * (L_z / nsections_z))
        mnode = fea.ChNodeFEAxyz(p)
        mesh.AddNode(mnode)
        mynodes.append(mnode)

# === Create elements ===
ementmonitor = None  # will be assigned when iz==0 and ix==1
for iz in range(nsections_z):
    for ix in range(nsections_x):
        # --- Element A ---
        melementA = fea.ChElementShellBST()
        mesh.AddElement(melementA)

        if iz == 0 and ix == 1:
            ementmonitor = melementA  # Monitor this element

        # Boundary nodes with conditional checks
        boundary_1 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]
        boundary_2 = mynodes[(iz + 1) * (nsections_x + 1) + ix - 1] if (ix > 0) else None
        boundary_3 = mynodes[(iz - 1) * (nsections_x + 1) + ix + 1] if (iz > 0) else None

        melementA.SetNodes(
            mynodes[iz * (nsections_x + 1) + ix],
            mynodes[iz * (nsections_x + 1) + ix + 1],
            mynodes[(iz + 1) * (nsections_x + 1) + ix],
            boundary_1, boundary_2, boundary_3
        )
        melementA.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)

        # --- Element B ---
        melementB = fea.ChElementShellBST()
        mesh.AddElement(melementB)

        boundary_1 = mynodes[iz * (nsections_x + 1) + ix]
        boundary_2 = mynodes[iz * (nsections_x + 1) + ix + 2] if (ix < nsections_x - 1) else None
        boundary_3 = mynodes[(iz + 2) * (nsections_x + 1) + ix] if (iz < nsections_z - 1) else None

        melementB.SetNodes(
            mynodes[(iz + 1) * (nsections_x + 1) + ix + 1],
            mynodes[(iz + 1) * (nsections_x + 1) + ix],
            mynodes[iz * (nsections_x + 1) + ix + 1],
            boundary_1, boundary_2, boundary_3
        )
        melementB.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)

# === Fix upper nodes ===
for j in range(30):
    for k in range(30):
        mynodes[j * (nsections_x + 1) + k].SetFixed(True)

# === Visualization shapes ===
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetShellResolution(2)
mvisualizeshellA.SetSmoothFaces(True)
mvisualizeshellA.SetWireframe(False)
# mvisualizeshellA.SetBackfaceCulling(False)  # optional backface culling
mesh.AddVisualShapeFEA(mvisualizeshellA)

mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetSymbolsThickness(0.006)
mesh.AddVisualShapeFEA(mvisualizeshellB)

# === Visualization (Irrlicht) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Shells FEA test: triangle BST elements")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 0.3, 1.3), chrono.ChVector3d(0.5, -0.3, 0.5))
vis.AddTypicalLights()

# === Solver ===
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)  # True per input2.txt
sys.SetSolver(mkl_solver)

# === Timestep ===
time_step = 0.005  # 0.005 per input2.txt

# === System setup ===
sys.Setup()
sys.Update()

# === Recording setup ===
REC = bool(os.environ.get("SIMBENCH_RECORD"))
irr_dir = rec.frame_dir("frames") if REC else None
os.makedirs("frames", exist_ok=True)

sim_end = 10.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

# === Main loop ===
frame = 0
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break

# === Review-only post-loop ===
