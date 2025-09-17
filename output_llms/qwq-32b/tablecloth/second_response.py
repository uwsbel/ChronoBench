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
melasticity = fea.ChElasticityKirchhoffIsotropic(E, nu)  # Fixed typo: Isotropic instead of Isothropic
material = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(density)

# Mesh dimensions
L_x, L_z = 1, 1
nsections_x, nsections_z = 40, 40

# Create nodes
mynodes = []
for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        p = chrono.ChVectorD(ix * (L_x / nsections_x), 0, iz * (L_z / nsections_z))
        mnode = fea.ChNodeFEAxyzD(p)  # Use ChNodeFEAxyzD for dynamic nodes
        mesh.AddNode(mnode)
        mynodes.append(mnode)

# Node Monitoring and Loading Setup
nodePlotA = mynodes[0]  # Example: first node
nodePlotB = mynodes[-1]  # Example: last node
nodesLoad = []
ref_X = lambda x: x  # Placeholder interpolation function
ref_Y = lambda y: y  # Placeholder interpolation function
load_force = chrono.ChVectorD(0, 0, 0)
mnodemonitor = mynodes[0]  # Example monitoring node
melementmonitor = None

# Fix upper nodes
for j in range(30):
    for k in range(30):
        node = mynodes[j * (nsections_x + 1) + k]
        node.SetFixed(True)

# Create elements
for iz in range(nsections_z):
    for ix in range(nsections_x):
        # Calculate boundary nodes with conditional checks
        boundary_1 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]
        boundary_2 = mynodes[(iz + 1) * (nsections_x + 1) + ix - 1] if ix > 0 else mynodes[(iz + 1) * (nsections_x + 1) + ix]
        boundary_3 = mynodes[(iz - 1) * (nsections_x + 1) + ix + 1] if iz > 0 else mynodes[(iz + 1) * (nsections_x + 1) + ix]

        melementA = fea.ChElementShellBST()
        melementA.SetNodes(
            mynodes[iz * (nsections_x + 1) + ix],
            mynodes[iz * (nsections_x + 1) + ix + 1],
            mynodes[(iz + 1) * (nsections_x + 1) + ix],
            boundary_1, boundary_2, boundary_3
        )
        melementA.AddLayer(thickness, 0, material)
        mesh.AddElement(melementA)

        if iz == 0 and ix == 1:
            melementmonitor = melementA  # Element monitoring assignment

        # Similar setup for melementB with adjusted boundaries
        boundary_1_b = mynodes[iz * (nsections_x + 1) + ix]
        boundary_2_b = mynodes[iz * (nsections_x + 1) + ix + 2] if ix < nsections_x - 1 else mynodes[iz * (nsections_x + 1) + ix + 1]
        boundary_3_b = mynodes[(iz + 2) * (nsections_x + 1) + ix] if iz < nsections_z - 1 else mynodes[(iz + 1) * (nsections_x + 1) + ix]

        melementB = fea.ChElementShellBST()
        melementB.SetNodes(
            mynodes[(iz + 1) * (nsections_x + 1) + ix + 1],
            mynodes[(iz + 1) * (nsections_x + 1) + ix],
            mynodes[iz * (nsections_x + 1) + ix + 1],
            boundary_1_b, boundary_2_b, boundary_3_b
        )
        melementB.AddLayer(thickness, 0, material)
        mesh.AddElement(melementB)

# Visualization setup
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetSmoothFaces(True)
mvisualizeshellA.SetWireframe(True)
# mvisualizeshellA.SetBackfaceCulling(True)  # Optional, commented out
mvisualizeshellA.SetShellResolution(2)
mesh.AddVisualShapeFEA(mvisualizeshellA)

mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetSymbolsThickness(0.006)
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  # Fixed data type
mesh.AddVisualShapeFEA(mvisualizeshellB)

# Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Shells FEA test: triangle BST elements')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1, 0.3, 1.3), chrono.ChVectorD(0.5, -0.3, 0.5))
vis.AddTypicalLights()

# Solver adjustment
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)  # Changed from False to True
sys.SetSolver(mkl_solver)

# Time step adjustment
timestep = 0.005

# Simulation setup
sys.Setup()
sys.Update()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(timestep)