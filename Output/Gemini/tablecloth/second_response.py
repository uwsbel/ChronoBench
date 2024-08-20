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
L_x, L_z = 1, 1
nsections_x, nsections_z = 40, 40

# --- Node Monitoring and Loading Setup ---
nodePlotA = None
nodePlotB = None
nodesLoad = []
# --- End of Node Monitoring and Loading Setup ---

# Create nodes
mynodes = []
for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        p = chrono.ChVector3d(ix * (L_x / nsections_x), 0, iz * (L_z / nsections_z))
        mnode = fea.ChNodeFEAxyz(p)
        mesh.AddNode(mnode)
        mynodes.append(mnode)

        # --- Node Monitoring and Loading Setup ---
        # Define reference points for nodes of interest
        if iz == 0 and ix == 1:
            nodePlotA = mnode
        if iz == int(nsections_z / 2) and ix == int(nsections_x / 2):
            nodePlotB = mnode
        if iz == int(nsections_z / 2) and ix == int(nsections_x / 2) + 1:
            nodesLoad.append(mnode)
        # --- End of Node Monitoring and Loading Setup ---

# Create elements
for iz in range(nsections_z):
    for ix in range(nsections_x):
        # Element A
        melementA = fea.ChElementShellBST()
        # --- Construct Boundary Nodes with Conditional Checks ---
        boundary_1 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]
        boundary_2 = mynodes[(iz + 1) * (nsections_x + 1) + ix - 1] if ix > 0 else None
        boundary_3 = mynodes[(iz - 1) * (nsections_x + 1) + ix + 1] if iz > 0 else None
        # --- End of Construct Boundary Nodes with Conditional Checks ---

        melementA.SetNodes(mynodes[iz * (nsections_x + 1) + ix], mynodes[iz * (nsections_x + 1) + ix + 1],
                           mynodes[(iz + 1) * (nsections_x + 1) + ix], boundary_1, boundary_2, boundary_3)
        melementA.AddLayer(thickness, 0, material)
        mesh.AddElement(melementA)

        # --- Element Monitoring ---
        if iz == 0 and ix == 1:
            melementmonitor = melementA
        # --- End of Element Monitoring ---

        # Element B
        melementB = fea.ChElementShellBST()
        # --- Construct Boundary Nodes with Conditional Checks ---
        boundary_1 = mynodes[iz * (nsections_x + 1) + ix]
        boundary_2 = mynodes[iz * (nsections_x + 1) + ix + 2] if ix < nsections_x - 1 else None
        boundary_3 = mynodes[(iz + 2) * (nsections_x + 1) + ix] if iz < nsections_z - 1 else None
        # --- End of Construct Boundary Nodes with Conditional Checks ---
        melementB.SetNodes(mynodes[(iz + 1) * (nsections_x + 1) + ix + 1], mynodes[(iz + 1) * (nsections_x + 1) + ix],
                           mynodes[iz * (nsections_x + 1) + ix + 1], boundary_1, boundary_2, boundary_3)
        melementB.AddLayer(thickness, 0, material)
        mesh.AddElement(melementB)

# --- Node Monitoring and Loading Setup ---
# Create interpolation functions
mfunA = fea.ChFunction_Recorder()
mfunB = fea.ChFunction_Recorder()
ref_X = chrono.ChFunction_Const(0.0)
ref_Y = chrono.ChFunction_Const(0.0)
# Load
load_force = chrono.ChVector3d(0, -0.5, 0)
# Monitoring
mnodemonitor = fea.ChNodeFEAxyz(chrono.ChVector3d(0, 0, 0))
melementmonitor = fea.ChElementShellBST()
# --- End of Node Monitoring and Loading Setup ---

# --- Fix Upper Nodes ---
for j in range(30):
    for k in range(30):
        mynodes[j * (nsections_x + 1) + k].SetFixed(True)
# --- End of Fix Upper Nodes ---

# Create visualizations for shell elements
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetShellResolution(2)
# mvisualizeshellA.SetSmoothFaces(True)
# mvisualizeshellA.SetWireframe(True)
# mvisualizeshellA.SetBackfaceCull(True)
mesh.AddVisualShapeFEA(mvisualizeshellA)

mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetSymbolsThickness(0.006)
# --- Adjust Visualization Shapes ---
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
# --- End of Adjust Visualization Shapes ---
mesh.AddVisualShapeFEA(mvisualizeshellB)

# Irrlicht visualization system setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Shells FEA test: triangle BST elements')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 0.3, 1.3), chrono.ChVector3d(0.5, -0.3, 0.5))
vis.AddTypicalLights()

# --- Solver Adjustment ---
# Change solver to PardisoMKL
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)  # Change to True for optimization
sys.SetSolver(mkl_solver)
# --- End of Solver Adjustment ---

# --- Time Step Adjustment ---
# Define time step for simulation
timestep = 0.005  # Changed from 0.001
# --- End of Time Step Adjustment ---

# Final setup and system updates
sys.Setup()
sys.Update()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(timestep)