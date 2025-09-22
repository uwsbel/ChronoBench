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

# Initialize node variables for plotting and loads
nodePlotA = None
nodePlotB = None
nodesLoad = []

# Create elements with boundary checks
for iz in range(nsections_z):
    for ix in range(nsections_x):
        # Elements
        melementA = fea.ChElementShellBST()
        boundary_1 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]
        boundary_2 = mynodes[(iz + 1) * (nsections_x + 1) + ix - 1] if ix > 0 else None
        boundary_3 = mynodes[(iz - 1) * (nsections_x + 1) + ix + 1] if iz > 0 else None

        melementA.SetNodes(
            mynodes[iz * (nsections_x + 1) + ix],
            mynodes[iz * (nsections_x + 1) + ix + 1],
            mynodes[(iz + 1) * (nsections_x + 1) + ix],
            boundary_1,
            boundary_2,
            boundary_3
        )
        melementA.AddLayer(thickness, 0, material)
        mesh.AddElement(melementA)

        melementB = fea.ChElementShellBST()
        boundary_1_b = mynodes[iz * (nsections_x + 1) + ix]
        boundary_2_b = mynodes[iz * (nsections_x + 1) + ix + 2] if ix < nsections_x - 1 else None
        boundary_3_b = mynodes[(iz + 2) * (nsections_x + 1) + ix] if iz < nsections_z - 1 else None

        melementB.SetNodes(
            mynodes[(iz + 1) * (nsections_x + 1) + ix + 1],
            mynodes[(iz + 1) * (nsections_x + 1) + ix],
            mynodes[iz * (nsections_x + 1) + ix + 1],
            boundary_1_b,
            boundary_2_b,
            boundary_3_b
        )
        melementB.AddLayer(thickness, 0, material)
        mesh.AddElement(melementB)

        # Assign monitoring node and element for reference if condition matches
        if iz == 0 and ix == 1:
            nodePlotA = mynodes[iz * (nsections_x + 1) + ix]
            nodePlotB = mynodes[iz * (nsections_x + 1) + ix + 1]
            nodesLoad = [mynodes[iz * (nsections_x + 1) + ix], mynodes[iz * (nsections_x + 1) + ix + 1]]

            ementmonitor = melementA  # Assign for monitoring purposes

# Fix certain upper nodes using nested loops
for j in range(30):
    for k in range(30):
        mynodes[j * (nsections_x + 1) + k].SetFixed(True)

# Create visualization shapes for shell elements
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetShellResolution(2)
mvisualizeshellA.SetSmoothFaces(True)
mvisualizeshellA.SetWireframe(False)
# Optional: mvisualizeshellA.SetBackfaceCull(True)  # Uncomment if backface culling is desired
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

# Change solver to PardisoMKL and lock sparsity pattern
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)  # Changed to True as per instructions
sys.SetSolver(mkl_solver)

# Define time step for simulation
timestep = 0.005  # Changed from 0.001 to 0.005

# Final system setup
sys.Setup()
sys.Update()

# Define reference functions (example placeholders)
def ref_X(t):
    return 0.5 * (1 + chrono.Sin(t))

def ref_Y(t):
    return 0.5 * (1 + chrono.Cos(t))

# Define load force vector
load_force = chrono.ChVector3d(0, -10, 0)  # Example downward force

# Assign node variables and load application
# (This part depends on how you want to apply/load forces; here is a placeholder)
# For example, applying a load to nodesLoad
for node in nodesLoad:
    # Applying load (assuming node has a method to add force)
    # Note: ChNodeFEAxyz may not have AddForce, so actual application depends on your setup
    # This is a conceptual placeholder
    node.SetForce(load_force)

# Create monitoring node and element if needed
# (Assuming monitoring node is a separate node, for example)
mnodemonitor = fea.ChNodeFEAxyz(chrono.ChVector3d(0, 0, 0))
mesh.AddNode(mnodemonitor)
# For monitoring element, assign if condition matches
if 'ementmonitor' in locals():
    # Example: attach a monitor or print info during simulation
    pass

# Run the simulation
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(timestep)