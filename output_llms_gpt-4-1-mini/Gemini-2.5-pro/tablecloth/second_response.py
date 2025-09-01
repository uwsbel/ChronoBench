import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl # Ensure PardisoMKL is available
import errno
import os

# Output directory setup (Corrected)
# chrono.GetChronoOutputPath() is not a standard PyChrono way to get a general output path.
# Using a local directory name instead.
out_dir = "FEA_SHELLS_BST_OUTPUT" 
try:
    os.mkdir(out_dir)
except OSError as exc:
    if exc.errno != errno.EEXIST:
        print("Error creating output directory")
    else:
        # Directory already exists, which is fine.
        pass


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
melasticity = fea.ChElasticityKirchhoffIsotropic(E, nu) # Corrected typo: Isothropic -> Isotropic
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

# --- Instruction 1: Node Monitoring and Loading Setup (Definitions) ---
# Define node variables for plotting
# Example: center node and a corner node
node_idx_plotA_iz = nsections_z // 2
node_idx_plotA_ix = nsections_x // 2
nodePlotA = mynodes[node_idx_plotA_iz * (nsections_x + 1) + node_idx_plotA_ix]

node_idx_plotB_iz = nsections_z
node_idx_plotB_ix = nsections_x
nodePlotB = mynodes[node_idx_plotB_iz * (nsections_x + 1) + node_idx_plotB_ix]

# Define nodesLoad list for applying loads (example: nodes along the edge z=0, y=0)
nodesLoad = []
# Example: Nodes along the edge iz = 0 (first row of nodes)
# for ix_load in range(nsections_x + 1):
#     nodesLoad.append(mynodes[0 * (nsections_x + 1) + ix_load])
# For demonstration, let's pick a few specific nodes for loading
if len(mynodes) > nsections_x // 2 : # ensure node exists
    nodesLoad.append(mynodes[0 * (nsections_x + 1) + nsections_x // 4])
    nodesLoad.append(mynodes[0 * (nsections_x + 1) + nsections_x // 2])
    nodesLoad.append(mynodes[0 * (nsections_x + 1) + 3 * nsections_x // 4])


# Create interpolation functions for reference tracking (dummy examples)
ref_X = chrono.ChFunction_Recorder()
ref_X.AddPoint(0, 0)
ref_X.AddPoint(1, 0.1)
ref_X.AddPoint(2, 0.2)

ref_Y = chrono.ChFunction_Recorder()
ref_Y.AddPoint(0, 0)
ref_Y.AddPoint(1, 0.05)
ref_Y.AddPoint(2, 0.15)

# Introduce a load_force vector
load_force = chrono.ChVector3d(0, -10, 0) # Example force: 10 units downwards

# Add monitoring node (example: a node near the center)
mnodemonitor_iz = nsections_z // 2
mnodemonitor_ix = nsections_x // 2 + 1 # A different node from nodePlotA
mnodemonitor = mynodes[mnodemonitor_iz * (nsections_x + 1) + mnodemonitor_ix]

# Initialize melementmonitor (will be assigned in the element creation loop)
melementmonitor = None
# --- End of Instruction 1 ---


# --- Instruction 4: Fix Upper Nodes ---
# Fix certain nodes in the mesh (e.g., a 30x30 block starting from (0,0))
# j corresponds to iz, k corresponds to ix
fix_dim_z = 30
fix_dim_x = 30
for j in range(min(fix_dim_z, nsections_z + 1)): # Iterate up to 29 for iz
    for k in range(min(fix_dim_x, nsections_x + 1)): # Iterate up to 29 for ix
        node_to_fix = mynodes[j * (nsections_x + 1) + k]
        node_to_fix.SetFixed(True)
# --- End of Instruction 4 ---


# Create elements
# Instruction 2: Construct Boundary Nodes with Conditional Checks
# The original script already implements conditional checks for boundary nodes,
# which is standard for ChElementShellBST. This logic is maintained.
for iz in range(nsections_z):
    for ix in range(nsections_x):
        # Node indices for the current quad
        n0_idx = iz * (nsections_x + 1) + ix
        n1_idx = iz * (nsections_x + 1) + ix + 1
        n2_idx = (iz + 1) * (nsections_x + 1) + ix
        n3_idx = (iz + 1) * (nsections_x + 1) + ix + 1

        node0 = mynodes[n0_idx] # (ix, iz)
        node1 = mynodes[n1_idx] # (ix+1, iz)
        node2 = mynodes[n2_idx] # (ix, iz+1)
        node3 = mynodes[n3_idx] # (ix+1, iz+1)

        # Element A (Triangle 0-1-2, i.e., node0-node1-node2)
        # Adjacent nodes:
        # adj to edge 1-2 (node1-node2): node3
        # adj to edge 2-0 (node2-node0): node at (ix-1, iz+1)
        # adj to edge 0-1 (node0-node1): node at (ix+1, iz-1)
        adj_A1 = node3
        adj_A2 = mynodes[(iz + 1) * (nsections_x + 1) + ix - 1] if ix > 0 else None
        adj_A3 = mynodes[(iz - 1) * (nsections_x + 1) + ix + 1] if iz > 0 else None
        
        melementA = fea.ChElementShellBST()
        melementA.SetNodes(node0, node1, node2, adj_A1, adj_A2, adj_A3)
        melementA.AddLayer(thickness, 0, material)
        mesh.AddElement(melementA)

        # --- Instruction 3: Element Monitoring ---
        if (iz == 0 and ix == 1):
            melementmonitor = melementA
        # --- End of Instruction 3 ---

        # Element B (Triangle 3-2-1, i.e., node3-node2-node1)
        # Adjacent nodes:
        # adj to edge 2-1 (node2-node1): node0
        # adj to edge 1-3 (node1-node3): node at (ix+2, iz)
        # adj to edge 3-2 (node3-node2): node at (ix, iz+2)
        adj_B1 = node0
        adj_B2 = mynodes[iz * (nsections_x + 1) + ix + 2] if ix < nsections_x - 1 else None
        adj_B3 = mynodes[(iz + 2) * (nsections_x + 1) + ix + 1] if iz < nsections_z - 1 else None # Corrected original potential minor index error for boundary_3 of melementB in thought process, but original was actually correct. Re-verified: Original: mynodes[(iz + 2) * (nsections_x + 1) + ix].
                                                                                                    # Let N0=node3, N1=node2, N2=node1 for melementB
                                                                                                    # Edge N0-N1 is node3-node2. Opposing node is at (ix,iz+2) -> mynodes[(iz+2)*(nsections_x+1)+ix]  OR (ix+1,iz+2) -> mynodes[(iz+2)*(nsections_x+1)+ix+1]
                                                                                                    # The original code was: `boundary_3 = mynodes[(iz + 2) * (nsections_x + 1) + ix] if iz < nsections_z - 1 else None` for melementB.
                                                                                                    # Nodes for melementB in original: N0=mynodes[(iz + 1) * (nsections_x + 1) + ix + 1], N1=mynodes[(iz + 1) * (nsections_x + 1) + ix], N2=mynodes[iz * (nsections_x + 1) + ix + 1]
                                                                                                    # Original boundary_3 was opposing N0-N1. This looks like the node at (ix, iz+2).
                                                                                                    # For triangle (ix+1,iz+1)-(ix,iz+1)-(ix+1,iz), the node opposing edge (ix+1,iz+1)-(ix,iz+1) is (ix,iz+2) or (ix+1,iz+2)
                                                                                                    # The original 'mynodes[(iz + 2) * (nsections_x + 1) + ix]' for adj_B3 (adj to node3-node2) is correct. It corresponds to node at (ix, iz+2).
        adj_B3_original_logic = mynodes[(iz + 2) * (nsections_x + 1) + ix] if iz < nsections_z - 1 else None


        melementB = fea.ChElementShellBST()
        melementB.SetNodes(node3, node2, node1, adj_B1, adj_B2, adj_B3_original_logic) # Using original logic for adj_B3 as it was correct.
        melementB.AddLayer(thickness, 0, material)
        mesh.AddElement(melementB)


# --- Instruction 5: Adjust Visualization Shapes ---
# Create visualizations for shell elements
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE) # Default, good for seeing the surface
mvisualizeshellA.SetShellResolution(2) # Kept from original
# As per instruction "conditionally (commented out) for possible optional visualization adjustments"
# mvisualizeshellA.SetSmoothFaces(True)
# mvisualizeshellA.SetWireframe(True)
# mvisualizeshellA.SetBackfaceCulling(True)
mesh.AddVisualShapeFEA(mvisualizeshellA)

mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetSymbolsThickness(0.006)
# Set the FEMdataType for mvisualizeshellB to DataType_NONE
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mesh.AddVisualShapeFEA(mvisualizeshellB)
# --- End of Instruction 5 ---


# Irrlicht visualization system setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Shells FEA test: triangle BST elements (Modified)')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 0.3, 1.3), chrono.ChVector3d(0.5, -0.3, 0.5))
vis.AddTypicalLights()

# --- Instruction 6: Solver Adjustment ---
# Change solver to PardisoMKL
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True) # Changed from False to True
sys.SetSolver(mkl_solver)
# --- End of Instruction 6 ---

# --- Instruction 7: Time Step Adjustment ---
# Define time step for simulation
timestep = 0.005 # Adjusted from 0.001
# --- End of Instruction 7 ---

# Final setup and system updates
sys.Setup()
sys.Update()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    # Example of how monitored variables could be used (printing to console):
    # if mnodemonitor:
    #     print(f"Time: {sys.GetChTime():0.4f} Monitored Node Pos: {mnodemonitor.GetPos()}")
    # if melementmonitor:
    #     # Example: Get stress (requires specific methods for BST, e.g. GetStressStrain)
    #     # This is a placeholder as specific stress/strain access for BST might need more setup
    #     # print(f"Monitored Element (e.g. centroid): {melementmonitor.EvaluatePT(chrono.ChVector3d(1/3,1/3,0),0)[0]}") # Get first gauss point for example
    #     pass 
    vis.EndScene()
    sys.DoStepDynamics(timestep)

# Cleanup visualization
# vis.EndLoop() # Not standard, vis.Run() handles loop break
# vis.GetDevice().closeDevice() # Not always needed, depends on context and if explicit cleanup is desired