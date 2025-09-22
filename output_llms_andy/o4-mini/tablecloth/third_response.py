import os
import errno

import pychrono.core as chrono           # Core Chrono functionality
import pychrono.irrlicht as chronoirr   # Irrlicht visualization
import pychrono.fea as fea              # FEA components
import pychrono.pardisomkl as mkl       # Pardiso MKL solver

# Output directory
out_dir = os.path.join(chrono.GetChronoOutputPath(), "FEA_SHELLS_BST")
os.makedirs(out_dir, exist_ok=True)

# Create the physical system
sys = chrono.ChSystemSMC()

# Create the mesh
mesh = fea.ChMesh()
sys.Add(mesh)
mesh.SetAutomaticGravity(False)

# (Unused placeholders: kept for completeness)
nodePlotA = fea.ChNodeFEAxyz()
nodePlotB = fea.ChNodeFEAxyz()
nodesLoad = []
ref_X = chrono.ChFunctionInterp()
ref_Y = chrono.ChFunctionInterp()
load_force = chrono.ChVector3d()
mnodemonitor = fea.ChNodeFEAxyz()
melementmonitor = None

# === SHELL BST MESH SETUP ===
density   = 100.0
E         = 6e4
nu        = 0.0
thickness = 0.01

melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)
material    = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(density)

L_x = 1.0
L_z = 1.0
nsections_x = 40
nsections_z = 40

# Create nodes
mynodes = []
for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        p = chrono.ChVector3d(ix * (L_x / nsections_x), 0.0, iz * (L_z / nsections_z))
        mnode = fea.ChNodeFEAxyz()
        mnode.SetXpos(p)
        mesh.AddNode(mnode)
        mynodes.append(mnode)

# Create triangular BST elements
for iz in range(nsections_z):
    for ix in range(nsections_x):
        # — first triangle —
        meA = fea.ChElementShellBST()
        mesh.AddElement(meA)
        if iz == 0 and ix == 1:
            melementmonitor = meA

        b1 = mynodes[(iz+1)*(nsections_x+1) + (ix+1)]
        b2 = mynodes[(iz+1)*(nsections_x+1) + (ix-1)] if ix > 0 else None
        b3 = mynodes[(iz-1)*(nsections_x+1) + (ix+1)] if iz > 0 else None

        meA.SetNodes(
            mynodes[iz*(nsections_x+1) + ix],
            mynodes[iz*(nsections_x+1) + ix+1],
            mynodes[(iz+1)*(nsections_x+1) + ix],
            b1, b2, b3
        )
        meA.AddLayer(thickness, 0.0 * chrono.CH_DEG_TO_RAD, material)

        # — second triangle —
        meB = fea.ChElementShellBST()
        mesh.AddElement(meB)
        b1 = mynodes[iz*(nsections_x+1) + ix]
        b2 = mynodes[iz*(nsections_x+1) + ix+2] if ix < nsections_x-1 else None
        b3 = mynodes[(iz+2)*(nsections_x+1) + ix] if iz < nsections_z-1 else None

        meB.SetNodes(
            mynodes[(iz+1)*(nsections_x+1) + ix+1],
            mynodes[(iz+1)*(nsections_x+1) + ix],
            mynodes[iz*(nsections_x+1) + ix+1],
            b1, b2, b3
        )
        meB.AddLayer(thickness, 0.0 * chrono.CH_DEG_TO_RAD, material)

# Fix a block of nodes in the corner
for j in range(30):
    for k in range(30):
        mynodes[j*(nsections_x+1) + k].SetFixed(True)

# === VISUALIZATION ===
# A) smooth/wireframe/back‐face‐culling visualization
mvisualizeshellA = chronoirr.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetSmoothFaces(True)
mvisualizeshellA.SetWireframe(True)
mvisualizeshellA.SetBackfaceCull(True)
mvisualizeshellA.SetShellResolution(2)
mesh.AddVisualShapeFEA(mvisualizeshellA)

# B) simple FEM‐glyph overlay
mvisualizeshellB = chronoirr.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetSymbolsThickness(0.006)
mesh.AddVisualShapeFEA(mvisualizeshellB)

# === IRRLICHT SETUP ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Shells FEA test: triangle BST elements')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, .3, 1.3), chrono.ChVector3d(.5, -.3, .5))
vis.AddTypicalLights()

# === SOLVER ===
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)
sys.SetSolver(mkl_solver)

# === INITIALIZE & RUN ===
timestep = 0.005
sys.Setup()
sys.Update()

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(timestep)

mvisualizeshellA.SetSmoothFaces(True)
     mvisualizeshellA.SetWireframe(True)
     mvisualizeshellA.SetBackfaceCull(True)