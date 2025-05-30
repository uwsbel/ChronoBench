import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr
import pychrono.pardisomkl as mkl # For solver, if explicitly creating solver object
import math
import os

# ---------------------------------------------------------------------
# Set Chrono data directory
# ---------------------------------------------------------------------
chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', '../../../chrono_data/'))

# ---------------------------------------------------------------------
# Create the Chrono physical system
# ---------------------------------------------------------------------
my_system = chrono.ChSystemSMC()
my_system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# ---------------------------------------------------------------------
# Create the FEA mesh
# ---------------------------------------------------------------------
my_mesh = fea.ChMesh()
my_system.Add(my_mesh)

# ---------------------------------------------------------------------
# Material properties for the tablecloth (isotropic Kirchhoff)
# ---------------------------------------------------------------------
# Fabric properties (can be tuned)
density = 150  # kg/m^3 (e.g., cotton/linen)
young_modulus = 5e5  # Pa (relatively flexible)
poisson_ratio = 0.3
thickness = 0.002  # 2mm
alpha_damping = 0.05 # Rayleigh damping alpha (mass proportional)
beta_damping = 0.001 # Rayleigh damping beta (stiffness proportional)


# Create the Kirchhoff material
material = fea.ChMaterialShellKirchhoff(density, young_modulus, poisson_ratio)
# Add Rayleigh damping to the material for numerical stability
material.SetRayleighDampingAlpha(alpha_damping)
material.SetRayleighDampingBeta(beta_damping)


# ---------------------------------------------------------------------
# Mesh grid parameters
# ---------------------------------------------------------------------
num_nodes_x = 15  # Number of nodes in X direction
num_nodes_z = 15  # Number of nodes in Z direction
size_x = 1.0  # Total length in X (meters)
size_z = 1.0  # Total length in Z (meters)
initial_height = 0.6 # Initial height of the tablecloth

# Store nodes in a 2D list for easy access
nodes_grid = [[None for _ in range(num_nodes_z)] for _ in range(num_nodes_x)]

# Add nodes to the mesh
print(f"Adding {num_nodes_x * num_nodes_z} nodes...")
for i in range(num_nodes_x):
    for j in range(num_nodes_z):
        # Calculate node position (centered at origin on XZ plane, lifted by initial_height)
        x = (i / (num_nodes_x - 1) - 0.5) * size_x
        y = initial_height
        z = (j / (num_nodes_z - 1) - 0.5) * size_z
        
        node = fea.ChNodeFEAxyz(chrono.ChVector3d(x, y, z))
        node.SetMass(0) # Mass will be added by elements
        my_mesh.AddNode(node)
        nodes_grid[i][j] = node

# Add shell elements to the mesh (ChElementShellReissner4 is a common choice)
print(f"Adding {(num_nodes_x - 1) * (num_nodes_z - 1)} elements...")
for i in range(num_nodes_x - 1):
    for j in range(num_nodes_z - 1):
        # Get the four nodes for the quadrilateral element
        node0 = nodes_grid[i][j]
        node1 = nodes_grid[i+1][j]
        node2 = nodes_grid[i+1][j+1]
        node3 = nodes_grid[i][j+1]
        
        element = fea.ChElementShellReissner4(thickness) # Use thickness here
        element.SetNodes(node0, node1, node2, node3)
        
        # Add a single layer with the defined material.
        # The angle 0 means fibers aligned with local element x-axis.
        # For isotropic material, this angle doesn't significantly matter.
        element.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)
        
        my_mesh.AddElement(element)

# ---------------------------------------------------------------------
# Boundary conditions to simulate folding (e.g., fix one edge)
# ---------------------------------------------------------------------
# Fix one edge of the tablecloth (nodes where i=0)
print("Applying boundary conditions (fixing one edge)...")
for j in range(num_nodes_z):
    nodes_grid[0][j].SetFixed(True)

# Optional: Add a rigid body for interaction (e.g., a table)
# For this example, we'll just let it drape in the air.

# ---------------------------------------------------------------------
# Solver Configuration - PardisoMKL
# ---------------------------------------------------------------------
# Use PardisoMKL solver for efficiency with FEA
# my_system.SetSolverType(chrono.ChSolver.Type_PARDISO_MKL)
# Or, more explicitly:
mkl_solver = mkl.ChSolverPardisoMKL()
my_system.SetSolver(mkl_solver)
mkl_solver.LockSparsityPattern(False) # Allow sparsity pattern to change if needed (e.g. contacts)
mkl_solver.SetVerbose(False)

# Set up the Timestepper (HHT is good for FEA)
my_system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
hht_stepper = my_system.GetTimestepper().StaticCast(chrono.ChTimestepperHHT)
if hht_stepper:
    hht_stepper.SetAlpha(-0.2)  # Numerical damping (typical: -0.2 to 0)
    hht_stepper.SetMaxiters(8)
    hht_stepper.SetAbsoler(1e-4)
    hht_stepper.SetReloler(1e-3)
    hht_stepper.SetMode(chrono.ChTimestepperHHT.POSITION) # Or ACCELERATION
    hht_stepper.SetStepControl(False) # Use fixed step size
    hht_stepper.SetVerbose(False)
else:
    print("Warning: Could not cast to ChTimestepperHHT. Using default HHT settings.")

# ---------------------------------------------------------------------
# Visualization setup - Irrlicht
# ---------------------------------------------------------------------
print("Setting up Irrlicht visualization...")
myapplication = irr.ChVisualSystemIrrlicht()
myapplication.AttachSystem(my_system)
myapplication.SetWindowSize(1024, 768)
myapplication.SetWindowTitle('PyChrono Tablecloth Folding (Shell Elements)')
myapplication.Initialize()
myapplication.AddLogo()
myapplication.AddSkyBox()
myapplication.AddCamera(chrono.ChVector3d(1.5, 1.0, 2.0)) # Camera position
myapplication.AddTypicalLights()

# Visualize the FEA mesh
vis_surface = chrono.ChVisualShapeFEA(my_mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_surface.SetColorscaleMinMax(0.0, 5.0) # Example scale for some data if shown
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False) # Set to True to see element outlines
my_mesh.AddVisualShapeFEA(vis_surface)

# Optional: visualize node normals or element outlines
# vis_nodes = chrono.ChVisualShapeFEA(my_mesh)
# vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
# vis_nodes.SetSymbolsDrawMode(chrono.ChVisualShapeFEA.SymbolType_DOT)
# vis_nodes.SetSymbolsScale(0.005)
# my_mesh.AddVisualShapeFEA(vis_nodes)

vis_wire = chrono.ChVisualShapeFEA(my_mesh)
vis_wire.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE) # Use surface type
vis_wire.SetWireframe(True) # But render as wireframe
vis_wire.SetDefaultODEColor(chrono.ChColor(0.1,0.1,0.1)) # Color for wireframe
my_mesh.AddVisualShapeFEA(vis_wire)


# ---------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------
timestep = 0.005  # Simulation timestep
simulation_time = 10.0 # Total simulation time

# Important: Finalize system setup before simulation
my_system.Setup()
my_system.Update()


print(f"Starting simulation for {simulation_time} seconds with timestep {timestep}...")
while myapplication.Run():
    myapplication.BeginScene(True, True, chrono.ChColor(0.2, 0.2, 0.3))
    myapplication.Render()
    
    # Draw some helper text
    # Note: Irrlicht font rendering might require specific setup or be version dependent.
    # For simplicity, we print to console.
    
    myapplication.EndScene()
    
    my_system.DoStepDynamics(timestep)
    
    # Print current time (optional)
    current_time = my_system.GetChTime()
    if int(current_time * 100) % 100 == 0 : # Print every second or so
         print(f"Time: {current_time:.2f} s")

    if current_time > simulation_time:
        myapplication.GetDevice().closeDevice() # Close window
        break

print("Simulation finished.")