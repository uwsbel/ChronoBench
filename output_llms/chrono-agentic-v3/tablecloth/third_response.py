"""
Tablecloth FEA Simulation — Triangle BST Shell Elements.

Models a tablecloth as a deformable thin shell using Kirchhoff BST elements on a
40×40 grid. Upper 30×30 nodes are fixed to simulate pinning. The simulation uses a
ChSystemSMC with a PardisoMKL solver (locked sparsity pattern) and automatic gravity.

Visualization enhancements on mvisualizeshellA:
  - SetSmoothFaces(True): smooth shading across triangle faces
  - SetWireframe(True): wireframe overlay on the shell surface
  - SetBackfaceCull(True): backface culling enabled
  - mvisualizeshellB FEMdataType set to DataType_NONE
  - Timestep: 0.005 s
"""

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import errno
import os

# === Simulation constants ===
density = 100           # kg/m³ — cloth density
E = 6e4                 # Pa — Young's modulus
nu = 0.0                # Poisson's ratio
thickness = 0.01        # m — shell thickness
L_x = 1                 # m — cloth length in X
L_z = 1                 # m — cloth length in Z
nsections_x = 40        # grid subdivisions in X
nsections_z = 40        # grid subdivisions in Z
timestep = 0.005        # s — time step
sim_end = 3.0           # s — total simulation duration
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * timestep)))  # precomputed once


# === Output directory ===
out_dir = chrono.GetChronoOutputPath() + "FEA_SHELLS_BST"
try:
    os.mkdir(out_dir)
except OSError as exc:
    if exc.errno != errno.EEXIST:
        print("Error creating output directory")

# === System & gravity ===
sys = chrono.ChSystemSMC()
# Pure FEA shell scene: no rigid-body contact — SetCollisionSystemType not required

# === FEA Mesh ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)
sys.Add(mesh)

# === Material ===
melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)
material = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(density)

# === Monitoring and load setup ===
nodePlotA = fea.ChNodeFEAxyz()     # node for plotting A
nodePlotB = fea.ChNodeFEAxyz()     # node for plotting B
nodesLoad = []                     # list of nodes with applied loads
ref_X = chrono.ChFunctionInterp()  # reference tracking X
ref_Y = chrono.ChFunctionInterp()  # reference tracking Y
load_force = chrono.ChVector3d()   # load force vector
mnodemonitor = fea.ChNodeFEAxyz()  # monitoring node
melementmonitor = fea.ChElementShellBST()  # monitoring element

# === Nodes ===
mynodes = []
for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        p = chrono.ChVector3d(
            ix * (L_x / nsections_x),
            0,
            iz * (L_z / nsections_z)
        )
        mnode = fea.ChNodeFEAxyz(p)
        mesh.AddNode(mnode)
        mynodes.append(mnode)

# === Elements (BST triangle pairs) ===
# FEA shell: no contact material needed — driven by gravity + shell elasticity only
ementmonitor = None
for iz in range(nsections_z):
    for ix in range(nsections_x):
        # First triangle element
        melementA = fea.ChElementShellBST()
        mesh.AddElement(melementA)

        if iz == 0 and ix == 1:
            ementmonitor = melementA  # assign monitoring element

        boundary_1 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]
        boundary_2 = mynodes[(iz + 1) * (nsections_x + 1) + ix - 1] if (ix > 0) else None
        boundary_3 = mynodes[(iz - 1) * (nsections_x + 1) + ix + 1] if (iz > 0) else None

        melementA.SetNodes(
            mynodes[(iz) * (nsections_x + 1) + ix],
            mynodes[(iz) * (nsections_x + 1) + ix + 1],
            mynodes[(iz + 1) * (nsections_x + 1) + ix],
            boundary_1, boundary_2, boundary_3
        )
        melementA.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)

        # Second triangle element
        melementB = fea.ChElementShellBST()
        mesh.AddElement(melementB)

        boundary_1 = mynodes[(iz) * (nsections_x + 1) + ix]
        boundary_2 = mynodes[(iz) * (nsections_x + 1) + ix + 2] if (ix < nsections_x - 1) else None
        boundary_3 = mynodes[(iz + 2) * (nsections_x + 1) + ix] if (iz < nsections_z - 1) else None

        melementB.SetNodes(
            mynodes[(iz + 1) * (nsections_x + 1) + ix + 1],
            mynodes[(iz + 1) * (nsections_x + 1) + ix],
            mynodes[(iz) * (nsections_x + 1) + ix + 1],
            boundary_1, boundary_2, boundary_3
        )
        melementB.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)

# === Fix upper nodes (30×30 pinned corner) ===
for j in range(30):
    for k in range(30):
        mynodes[j * (nsections_x + 1) + k].SetFixed(True)

# === FEA Visualization shapes ===
# Shape A: smooth wireframe with backface culling
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetSmoothFaces(True)       # enable smooth faces
mvisualizeshellA.SetWireframe(True)         # enable wireframe mode
mvisualizeshellA.SetBackfaceCull(True)      # enable backface culling
mvisualizeshellA.SetShellResolution(2)
mesh.AddVisualShapeFEA(mvisualizeshellA)

# Shape B: node dot glyphs
mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetSymbolsThickness(0.006)
mesh.AddVisualShapeFEA(mvisualizeshellB)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Shells FEA test: triangle BST elements')
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 0.3, 1.3), chrono.ChVector3d(0.5, -0.3, 0.5))
vis.AddTypicalLights()

# === Solver: PardisoMKL with locked sparsity pattern ===
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)   # locked for computational efficiency
sys.SetSolver(mkl_solver)

# === Pre-simulation system setup ===
sys.Setup()
sys.Update()

# Interpolation records for post-processing reference
rec_X = chrono.ChFunctionInterp()
rec_Y = chrono.ChFunctionInterp()


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
except (RuntimeError, ValueError) as exc:  # solver divergence / bad mesh state
    import traceback
    traceback.print_exc()
    raise
