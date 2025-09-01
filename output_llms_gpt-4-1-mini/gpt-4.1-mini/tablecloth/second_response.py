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

# Create nodes
mynodes = []
for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        p = chrono.ChVector3d(ix * (L_x / nsections_x), 0, iz * (L_z / nsections_z))
        mnode = fea.ChNodeFEAxyz(p)
        mesh.AddNode(mnode)
        mynodes.append(mnode)

# Initialize monitoring and load nodes lists
nodePlotA = []
nodePlotB = []
nodesLoad = []

# Interpolation functions for reference tracking
# Linear interpolation functions defining a reference axis along X and Z directions
# Using Python lambdas fitting linear interpolation from points (0,0) to (1,1)
ref_X = lambda x: x  # Identity function as position.x / L_x is direct ratio 0..1
ref_Y = lambda z: z  # Identity function as position.z / L_z is direct ratio 0..1

# Define load force vector to be applied on load nodes
load_force = chrono.ChVectorD(0, -10, 0)  # Load in negative Y direction (e.g. gravity-like)

# Create elements
melementmonitor = None  # Placeholder for monitoring element
for iz in range(nsections_z):
    for ix in range(nsections_x):
        melementA = fea.ChElementShellBST()

        # Safe boundary node assignments with conditional checks
        boundary_1 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]

        boundary_2 = (
            mynodes[(iz + 1) * (nsections_x + 1) + ix - 1] if ix > 0 else None
        )
        boundary_3 = (
            mynodes[(iz - 1) * (nsections_x + 1) + ix + 1] if iz > 0 else None
        )

        melementA.SetNodes(
            mynodes[iz * (nsections_x + 1) + ix],
            mynodes[iz * (nsections_x + 1) + ix + 1],
            mynodes[(iz + 1) * (nsections_x + 1) + ix],
            boundary_1,
            boundary_2,
            boundary_3,
        )
        melementA.AddLayer(thickness, 0, material)
        mesh.AddElement(melementA)

        # Assign melementA to monitor if condition matches
        if iz == 0 and ix == 1:
            melementmonitor = melementA

        melementB = fea.ChElementShellBST()

        boundary_1_b = mynodes[iz * (nsections_x + 1) + ix]
        boundary_2_b = (
            mynodes[iz * (nsections_x + 1) + ix + 2] if ix < nsections_x - 1 else None
        )
        boundary_3_b = (
            mynodes[(iz + 2) * (nsections_x + 1) + ix] if iz < nsections_z - 1 else None
        )

        melementB.SetNodes(
            mynodes[(iz + 1) * (nsections_x + 1) + ix + 1],
            mynodes[(iz + 1) * (nsections_x + 1) + ix],
            mynodes[iz * (nsections_x + 1) + ix + 1],
            boundary_1_b,
            boundary_2_b,
            boundary_3_b,
        )
        melementB.AddLayer(thickness, 0, material)
        mesh.AddElement(melementB)

# Define monitoring nodes and loaded nodes
# Usually some nodes near the center or edges for monitoring and some for applying loads

# Example: populate nodePlotA and nodePlotB along a line on mesh surface for tracking
# Here we pick nodes on the middle at ground level and top level respectively

mid_iz = nsections_z // 2
mid_ix = nsections_x // 2

nodePlotA.append(mynodes[mid_iz * (nsections_x + 1) + mid_ix])
nodePlotB.append(mynodes[(mid_iz + 10) * (nsections_x + 1) + mid_ix + 5])

# Example nodes for loads - let's pick 4 corner nodes to apply load forces
nodesLoad.append(mynodes[0])  # corner (0,0)
nodesLoad.append(mynodes[nsections_x])  # corner (nsections_x,0)
nodesLoad.append(mynodes[-1])  # corner (nsections_x,nsections_z)
nodesLoad.append(mynodes[-1 - nsections_x])  # corner (0,nsections_z)

# Apply load with Chrono load container or force elements

# Create a load container
loads = chrono.ChLoadContainer()
sys.Add(loads)

for node in nodesLoad:
    # Add force at each node from centralized load_force
    force_load = chrono.ChLoadBodyForce(node, False, load_force)
    loads.Add(force_load)

# Create monitoring nodes/elements for plotting or output - for example monitor displacement
# We can keep references to them, or add custom functions later for export or analysis

# Fix upper nodes loop as requested: fix nodes in a 30x30 grid (clamped boundary)
for j in range(30):
    for k in range(30):
        idx = j * (nsections_x + 1) + k
        if idx < len(mynodes):
            mynodes[idx].SetFixed(True)

# Create visualizations for shell elements
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetShellResolution(2)

# Optional visualization adjustments (commented out as per instruction)
# mvisualizeshellA.SetSmoothFaces(True)
# mvisualizeshellA.SetWireframe(True)
# mvisualizeshellA.SetBackfaceCull(False)

mesh.AddVisualShapeFEA(mvisualizeshellA)

mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetSymbolsThickness(0.006)
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
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

# Change solver to PardisoMKL
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)  # Change from False to True as per instruction
sys.SetSolver(mkl_solver)

# Define time step for simulation
timestep = 0.005  # Changed from 0.001 to 0.005

# Final setup and system updates
sys.Setup()
sys.Update()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(timestep)