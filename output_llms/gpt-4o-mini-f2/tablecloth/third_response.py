import pychrono.core as chrono  # Import core Chrono functionality
import pychrono.irrlicht as chronoirr  # Import Irrlicht visualization
import pychrono.fea as fea  # Import Finite Element Analysis components
import pychrono.pardisomkl as mkl  # Import PardisoMKL solver
import errno  # Import standard error numbers
import os  # Import system operations for file handling

# Output directory
out_dir = chrono.GetChronoOutputPath() + "FEA_SHELLS_BST"  # Define the output directory path

# Create (if needed) the output directory
try:
    os.mkdir(out_dir)  # Try to create the directory
except OSError as exc:
    if exc.errno != errno.EEXIST:  # If directory exists, no error; otherwise, print error message
        print("Error creating output directory ")

# Create a Chrono::Engine physical system
sys = chrono.ChSystemSMC()  # Instantiate the physical system

# Create a mesh, a container for groups of elements and their referenced nodes
mesh = fea.ChMesh()  # Instantiate the mesh

# Add the created mesh to the physical system
sys.Add(mesh)

# Disable gravity for the system (options)
# sys.SetGravitationalAcceleration(VNULL) or
mesh.SetAutomaticGravity(False)

# Define nodes to plot or load
nodePlotA = fea.ChNodeFEAxyz()  # Node for plotting
nodePlotB = fea.ChNodeFEAxyz()  # Another node for plotting
nodesLoad = []  # List for nodes to apply loads

# Create interpolation functions for reference tracking (if needed)
ref_X = chrono.ChFunctionInterp()
ref_Y = chrono.ChFunctionInterp()

# Define load force vector
load_force = chrono.ChVector3d()

# Monitoring nodes and elements
mnodemonitor = fea.ChNodeFEAxyz()  # Node for monitoring
melementmonitor = fea.ChElementShellBST()  # Element for monitoring

if (True):  # Block to execute the following setup
    # Define material properties
    density = 100  # Material density
    E = 6e4  # Young's modulus
    nu = 0.0  # Poisson's ratio
    thickness = 0.01  # Thickness of the shell

    # Create isotropic Kirchhoff material elasticity object
    melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)

    # Create material object by assigning the elasticity property
    material = fea.ChMaterialShellKirchhoff(melasticity)
    material.SetDensity(density)  # Set density of the material

    # Define the mesh dimensions
    L_x = 1  # Length in x direction
    nsections_x = 40  # Number of sections in x direction
    L_z = 1  # Length in z direction
    nsections_z = 40  # Number of sections in z direction

    # Create list to hold the nodes
    mynodes = []

    # Create nodes for the mesh grid
    for iz in range(nsections_z + 1):
        for ix in range(nsections_x + 1):
            p = chrono.ChVector3d(ix * (L_x / nsections_x), 0, iz * (L_z / nsections_z))  # Calculate node position
            mnode = fea.ChNodeFEAxyz(p)  # Create node at calculated position
            mesh.AddNode(mnode)  # Add node to mesh
            mynodes.append(mnode)  # Append node to the list

    # Create elements and associate nodes
    for iz in range(nsections_z):
        for ix in range(nsections_x):
            # Create first element
            melementA = fea.ChElementShellBST()
            mesh.AddElement(melementA)

            if (iz == 0 and ix == 1):
                ementmonitor = melementA  # Assign element to monitor

            # Define boundary nodes
            boundary_1 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]
            boundary_2 = mynodes[(iz + 1) * (nsections_x + 1) + ix - 1] if (ix > 0) else None
            boundary_3 = mynodes[(iz - 1) * (nsections_x + 1) + ix + 1] if (iz > 0) else None

            # Set nodes to the element
            melementA.SetNodes(
                mynodes[(iz) * (nsections_x + 1) + ix],
                mynodes[(iz) * (nsections_x + 1) + ix + 1],
                mynodes[(iz + 1) * (nsections_x + 1) + ix],
                boundary_1, boundary_2, boundary_3
            )

            # Add layer to the element
            melementA.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)

            # Create second element
            melementB = fea.ChElementShellBST()
            mesh.AddElement(melementB)

            # Define boundary nodes
            boundary_1 = mynodes[(iz) * (nsections_x + 1) + ix]
            boundary_2 = mynodes[(iz) * (nsections_x + 1) + ix + 2] if (ix < nsections_x - 1) else None
            boundary_3 = mynodes[(iz + 2) * (nsections_x + 1) + ix] if (iz < nsections_z - 1) else None

            # Set nodes to the element
            melementB.SetNodes(
                mynodes[(iz + 1) * (nsections_x + 1) + ix + 1],
                mynodes[(iz + 1) * (nsections_x + 1) + ix],
                mynodes[(iz) * (nsections_x + 1) + ix + 1],
                boundary_1, boundary_2, boundary_3
            )

            # Add layer to the element
            melementB.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)

    # Fix upper nodes of the mesh
    for j in range(30):
        for k in range(30):
            mynodes[j * (nsections_x + 1) + k].SetFixed(True)

    # Create visualizations for shell elements
    mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
    mvisualizeshellA.SetSmoothFaces(True)  # Enable smooth faces (optional)
    mvisualizeshellA.SetWireframe(True)  # Enable wireframe mode (optional)
    mvisualizeshellA.SetShellResolution(2)  # Set resolution for shell visualization
    mvisualizeshellA.SetBackfaceCull(True)  # Enable backface culling (optional)
    mesh.AddVisualShapeFEA(mvisualizeshellA)  # Add visual shape to mesh

    # Create visualization of FEM data
    mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
    mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  # Set visual type
    mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  # Set glyph type
    mvisualizeshellB.SetSymbolsThickness(0.006)  # Set thickness of symbols
    mesh.AddVisualShapeFEA(mvisualizeshellB)  # Add visual shape to mesh

# Create Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  # Attach the physical system to the visualization
vis.SetWindowSize(1024, 768)  # Set window size
vis.SetWindowTitle('Shells FEA test: triangle BST elements')  # Set window title
vis.Initialize()  # Initialize the visualization window
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add logo to the window
vis.AddSkyBox()  # Add a skybox for background
vis.AddCamera(chrono.ChVector3d(1, .3, 1.3), chrono.ChVector3d(.5, -.3, .5))  # Add and position the camera
vis.AddTypicalLights()  # Add typical lights to the scene

# Change solver to PardisoMKL
mkl_solver = mkl.ChSolverPardisoMKL()  # Instantiate PardisoMKL solver
mkl_solver.LockSparsityPattern(True)  # Locks sparsity pattern of the matrix for optimization
sys.SetSolver(mkl_solver)  # Set the solver in the system

# Define time step for simulation
timestep = 0.005

# Final setup and system updates
sys.Setup()  # Prepare the system for simulation
sys.Update()  # Update the system's internal states

# Initialize interpolation function records (if required)
rec_X = chrono.ChFunctionInterp()
rec_Y = chrono.ChFunctionInterp()

# Simulation loop for running the visualization and dynamical system
while vis.Run():
    vis.BeginScene()  # Begin the visualization scene
    vis.Render()  # Render the scene
    vis.EndScene()  # End the visualization scene

    sys.DoStepDynamics(timestep)  # Perform a simulation step with the defined timestep