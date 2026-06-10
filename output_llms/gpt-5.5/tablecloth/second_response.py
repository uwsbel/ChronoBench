import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import errno
import os

# Output directory setup
out_dir = chrono.GetChronoOutputPath() + "FEA_SHELLS_BST"
try:
    os.mkdir(out_dir)
except OSError as exc:
    if exc.errno != errno.EEXIST:
        print("Error creating output directory")

# Create Chrono physical system
sys = chrono.ChSystemSMC()

# Create and add mesh to the system
mesh = fea.ChMesh()
sys.Add(mesh)

# Material properties
density = 100
E = 6e4
nu = 0.0
thickness = 0.01

# Create material
melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)
material = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(density)

# Mesh dimensions
L_x, L_z = 1.0, 1.0
nsections_x, nsections_z = 40, 40

# -------------------------------------------------------------------------
# Node monitoring and loading setup
# -------------------------------------------------------------------------
nodePlotA = None
nodePlotB = None
nodesLoad = []

mnodemonitor = None
melementmonitor = None

# Optional reference interpolation functions for tracking/comparison
ref_X = chrono.ChFunctionInterp()
ref_X.AddPoint(0.0, 0.0)
ref_X.AddPoint(1.0, 0.0)

ref_Y = chrono.ChFunctionInterp()
ref_Y.AddPoint(0.0, 0.0)
ref_Y.AddPoint(1.0, 0.0)

# Load force applied to selected nodes
load_force = chrono.ChVector3d(0.0, -0.2, 0.0)

# -------------------------------------------------------------------------
# Create nodes
# -------------------------------------------------------------------------
mynodes = []
for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        p = chrono.ChVector3d(
            ix * (L_x / nsections_x),
            0.0,
            iz * (L_z / nsections_z)
        )
        mnode = fea.ChNodeFEAxyz(p)
        mesh.AddNode(mnode)
        mynodes.append(mnode)

# Define plotting / monitoring nodes after all nodes are created
nodePlotA = mynodes[nsections_z * (nsections_x + 1) + nsections_x]
nodePlotB = mynodes[(nsections_z // 2) * (nsections_x + 1) + nsections_x]

mnodemonitor = nodePlotA

# Nodes where external loads are applied
nodesLoad = [nodePlotA, nodePlotB]
for node in nodesLoad:
    node.SetForce(load_force)

# -------------------------------------------------------------------------
# Fix selected nodes
# -------------------------------------------------------------------------
for j in range(30):
    for k in range(30):
        mynodes[j * (nsections_x + 1) + k].SetFixed(True)

# -------------------------------------------------------------------------
# Create BST shell elements
# -------------------------------------------------------------------------
for iz in range(nsections_z):
    for ix in range(nsections_x):

        # First triangle in the quad
        melementA = fea.ChElementShellBST()

        boundary_1 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]

        if ix > 0:
            boundary_2 = mynodes[(iz + 1) * (nsections_x + 1) + ix - 1]
        else:
            boundary_2 = None

        if iz > 0:
            boundary_3 = mynodes[(iz - 1) * (nsections_x + 1) + ix + 1]
        else:
            boundary_3 = None

        melementA.SetNodes(
            mynodes[iz * (nsections_x + 1) + ix],
            mynodes[iz * (nsections_x + 1) + ix + 1],
            mynodes[(iz + 1) * (nsections_x + 1) + ix],
            boundary_1,
            boundary_2,
            boundary_3
        )
        melementA.AddLayer(thickness, 0.0, material)
        mesh.AddElement(melementA)

        # Assign element monitor at requested element location
        if iz == 0 and ix == 1:
            melementmonitor = melementA

        # Second triangle in the quad
        melementB = fea.ChElementShellBST()

        boundary_1 = mynodes[iz * (nsections_x + 1) + ix]

        if ix < nsections_x - 1:
            boundary_2 = mynodes[iz * (nsections_x + 1) + ix + 2]
        else:
            boundary_2 = None

        if iz < nsections_z - 1:
            boundary_3 = mynodes[(iz + 2) * (nsections_x + 1) + ix]
        else:
            boundary_3 = None

        melementB.SetNodes(
            mynodes[(iz + 1) * (nsections_x + 1) + ix + 1],
            mynodes[(iz + 1) * (nsections_x + 1) + ix],
            mynodes[iz * (nsections_x + 1) + ix + 1],
            boundary_1,
            boundary_2,
            boundary_3
        )
        melementB.AddLayer(thickness, 0.0, material)
        mesh.AddElement(melementB)

# -------------------------------------------------------------------------
# Visualization for shell elements
# -------------------------------------------------------------------------
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
mvisualizeshellA.SetShellResolution(2)
mvisualizeshellA.SetSmoothFaces(True)
mvisualizeshellA.SetWireframe(False)
# Optional visualization adjustment:
# mvisualizeshellA.SetBackfaceCull(False)
mesh.AddVisualShapeFEA(mvisualizeshellA)

mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetSymbolsThickness(0.006)
mesh.AddVisualShapeFEA(mvisualizeshellB)

# -------------------------------------------------------------------------
# Irrlicht visualization system setup
# -------------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Shells FEA test: triangle BST elements")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(1.0, 0.3, 1.3),
    chrono.ChVector3d(0.5, -0.3, 0.5)
)
vis.AddTypicalLights()

# -------------------------------------------------------------------------
# Solver setup: PardisoMKL
# -------------------------------------------------------------------------
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)
sys.SetSolver(mkl_solver)

# Adjusted time step
timestep = 0.005

# Final setup and system updates
sys.Setup()
sys.Update()

# -------------------------------------------------------------------------
# Simulation loop
# -------------------------------------------------------------------------
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Keep nodal forces applied during the simulation
    for node in nodesLoad:
        node.SetForce(load_force)

    sys.DoStepDynamics(timestep)