# Set up the system and output directory
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import errno
import os

# Output directory
out_dir = chrono.GetChronoOutputPath() + "FEA_SHELLS_BST"

# Create (if needed) the output directory
try:
    os.mkdir(out_dir)
except OSError as exc:
    if exc.errno != errno.EEXIST:
        print("Error creating output directory ")

# Create a Chrono::Engine physical system
sys = chrono.ChSystemSMC()

# Create a mesh, a container for groups of elements and their referenced nodes
mesh = fea.ChMesh();

# Add the created mesh to the physical system
sys.Add(mesh);

# Define nodes to monitor and plot
nodePlotA = fea.ChNodeFEAxyz()
nodePlotB = fea.ChNodeFEAxyz()
nodesLoad = []

# Create interpolation functions for reference tracking (if needed)
ref_X = chrono.ChFunctionInterp()
ref_Y = chrono.ChFunctionInterp()

# Define load force vector
load_force = chrono.ChVector3d()

# Provide some visualization data types for the mesh
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SHELL painless mass)
mvisualizeshellA.SetZbufferHide(True)
mesh.AddVisualShapeFEA(mvisualizeshellA)

mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizeshellB.SetSymbolsThickness(0.006)
mvisualizeshellB.SetZbufferHide(True)
mesh.AddVisualShapeFEA(mvisualizeshellB)

# Change resolver to PardisoMKL instead of the default KKT fullSparsityPattern
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)  # Locks sparsity pattern of the stiffness matrix for optimization
sys.ChangeSolver(mkl_solver)

# Define mesh parameters
density = 100
E = 6e4
nu = 0.0
thickness = 0.01
L_x = 1
nsections_x = 40
L_z = 1
nsections_z = 40

# Create material object by defining its properties
melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)
material = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(density)

# Create nodes for the mesh grid
mynodes = []
for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        p = chrono.ChVector3d(ix * (L_x / nsections_x), 0, iz * (L_z / nsections_z))
        mnode = fea.ChNodeFEAxyz(p)
        mesh.AddNode(mnode)
        mynodes.append(mnode)

# Create elements for the mesh
for iz in range(nsections_z):
    for ix in range(nsections_x):
        # Create first triangle
        melementA = fea.ChElementShellBST()
        if (iz == 0 and ix == 1):
            ementmonitor = melementA  # Assign element to monitor
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
        mesh.AddElement(melementA)

        # Create second triangle
        melementB = fea.ChElementShellBST()
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
        mesh.AddElement(melementB)


# Fix upper nodes of the mesh
for j in range(30):
    for k in range(30):
        mynodes[j * (nsections_x + 1) + k].SetFixed(True)
        mnodemonitor = mynodes[(nsections_z) * (nsections_x + 1) + 1]


# Create interpolation functions for reference tracking (if needed)
ref_X = chrono.ChFunctionInterp()
ref_Y = chrono.ChFunctionInterp()

# Define load force vector
load_force = chrono.ChVector3d()

# Provide some visualization data types for the mesh
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SHELL)
mvisualizeshellA.SetZbufferHide(True)
mesh.AddVisualShapeFEA(mvisualizeshellA)

mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizeshellB.SetSymbolsThickness(0.006)
mvisualizeshellB.SetZbufferHide(True)
mesh.AddVisualShapeFEA(mvisualizeshellB)

# Irrlicht visualization system setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Shells FEA test: triangle BST elements')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1,0.3,1.3), chrono.ChVector3d(0.5,-0.3,0.5))
vis.AddTypicalLights()

# Change solver to PardisoMKL
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)  # Locks sparsity pattern of the stiffness matrix for optimization
sys.SetSolver(mkl_solver)

# Define time step for simulation
timestep = 0.005

# Final setup and system updates
sys.Setup()
sys.Update()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(timestep)