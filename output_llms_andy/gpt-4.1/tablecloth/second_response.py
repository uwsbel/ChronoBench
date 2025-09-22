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

# 1. Node Monitoring and Loading Setup
# Define node variables for plotting and loading
nodePlotA = mynodes[0]  # Example: first node
nodePlotB = mynodes[-1]  # Example: last node
nodesLoad = []  # Will be filled below

# Interpolation functions for reference tracking (example: identity functions)
def ref_X(t): return t
def ref_Y(t): return t

# Define load force vector (example: force in Y direction)
load_force = chrono.ChVector3d(0, -100, 0)

# Monitoring node and element (to be assigned later)
mnodemonitor = None
melementmonitor = None

# Create elements
for iz in range(nsections_z):
    for ix in range(nsections_x):
        # --- Element A ---
        melementA = fea.ChElementShellBST()
        # Boundary nodes with conditional checks to avoid out-of-bounds
        boundary_1 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1] if (ix + 1 <= nsections_x and iz + 1 <= nsections_z) else None
        boundary_2 = mynodes[(iz + 1) * (nsections_x + 1) + ix - 1] if (ix > 0 and iz + 1 <= nsections_z) else None
        boundary_3 = mynodes[(iz - 1) * (nsections_x + 1) + ix + 1] if (iz > 0 and ix + 1 <= nsections_x) else None

        # Set nodes for element A
        melementA.SetNodes(
            mynodes[iz * (nsections_x + 1) + ix],
            mynodes[iz * (nsections_x + 1) + ix + 1],
            mynodes[(iz + 1) * (nsections_x + 1) + ix],
            boundary_1, boundary_2, boundary_3
        )
        melementA.AddLayer(thickness, 0, material)
        mesh.AddElement(melementA)

        # 3. Element Monitoring: assign element for monitoring
        if iz == 0 and ix == 1:
            melementmonitor = melementA

        # --- Element B ---
        melementB = fea.ChElementShellBST()
        boundary_1 = mynodes[iz * (nsections_x + 1) + ix] if (iz * (nsections_x + 1) + ix < len(mynodes)) else None
        boundary_2 = mynodes[iz * (nsections_x + 1) + ix + 2] if (ix < nsections_x - 1 and iz * (nsections_x + 1) + ix + 2 < len(mynodes)) else None
        boundary_3 = mynodes[(iz + 2) * (nsections_x + 1) + ix] if (iz < nsections_z - 1 and (iz + 2) * (nsections_x + 1) + ix < len(mynodes)) else None

        melementB.SetNodes(
            mynodes[(iz + 1) * (nsections_x + 1) + ix + 1],
            mynodes[(iz + 1) * (nsections_x + 1) + ix],
            mynodes[iz * (nsections_x + 1) + ix + 1],
            boundary_1, boundary_2, boundary_3
        )
        melementB.AddLayer(thickness, 0, material)
        mesh.AddElement(melementB)

# 4. Fix Upper Nodes
for j in range(30):
    for k in range(30):
        idx = j * (nsections_x + 1) + k
        if idx < len(mynodes):
            mynodes[idx].SetFixed(True)

# 1. Node Monitoring and Loading Setup (continued)
# Example: pick a node in the middle for monitoring and loading
mid_ix = nsections_x // 2
mid_iz = nsections_z // 2
mid_idx = mid_iz * (nsections_x + 1) + mid_ix
if mid_idx < len(mynodes):
    mnodemonitor = mynodes[mid_idx]
    nodesLoad.append(mynodemonitor)

# 2. (already handled above) - boundary nodes with conditional checks

# 5. Adjust Visualization Shapes
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetShellResolution(2)
# Optional visualization settings (uncomment to enable)
# mvisualizeshellA.SetSmoothFaces(True)
# mvisualizeshellA.SetWireframe(True)
# mvisualizeshellA.SetBackfaceCulling(True)
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

# 6. Change solver to PardisoMKL and lock sparsity pattern
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)  # Changed from False to True
sys.SetSolver(mkl_solver)

# 7. Adjust time step for simulation
timestep = 0.005  # Changed from 0.001 to 0.005

# Final setup and system updates
sys.Setup()
sys.Update()

# Example: Apply load to nodes in nodesLoad
for node in nodesLoad:
    load = fea.ChLoadNodeXYZ(node, load_force)
    mesh.AddLoad(load)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(timestep)