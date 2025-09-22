import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr
import numpy as np

# Global simulation parameters
cloth_size_x = 1.0  # m
cloth_size_y = 1.0  # m
thickness = 0.01  # m
density = 200.0  # kg/m^3
E = 0.5e6  # Young's modulus (Pa)
nu = 0.3  # Poisson ratio
mesh_resolution = 20  # Number of elements in each direction

# Initialize the simulation system
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create a material for the tablecloth (Kirchhoff-Love shell element)
material = fea.ChMaterialShellKirchhoff()
material.SetDensity(density)
material.SetYoungModulus(E)
material.SetPoissonRatio(nu)

# Create the mesh
mesh = fea.ChMesh()

# Helper function to create the shell element mesh
def create_tablecloth_mesh(size_x, size_y, resolution, thickness, material):
    # Create nodes and elements
    nodes = []
    elements = []
    
    dx = size_x / resolution
    dy = size_y / resolution
    
    # Create nodes
    for iy in range(resolution + 1):
        for ix in range(resolution + 1):
            x = ix * dx - size_x/2
            y = 0.5  # Initial height
            z = iy * dy - size_y/2
            
            node = fea.ChNodeFEAxyz(chrono.ChVectorD(x, y, z))
            mesh.AddNode(node)
            nodes.append(node)
    
    # Create elements
    for iy in range(resolution):
        for ix in range(resolution):
            # Get the four nodes for this element
            idx1 = ix + iy * (resolution + 1)
            idx2 = ix + 1 + iy * (resolution + 1)
            idx3 = ix + 1 + (iy + 1) * (resolution + 1)
            idx4 = ix + (iy + 1) * (resolution + 1)
            
            # Create the shell element
            element = fea.ChElementShellKirchhoff4()
            element.SetNodes(nodes[idx1], nodes[idx2], nodes[idx3], nodes[idx4])
            element.SetDimensions(dx, dy)
            element.AddLayer(thickness, 0, material)
            
            # Add element to mesh
            mesh.AddElement(element)
            elements.append(element)
    
    return nodes, elements

# Create the tablecloth mesh
nodes, elements = create_tablecloth_mesh(cloth_size_x, cloth_size_y, mesh_resolution, thickness, material)

# Fix some nodes to simulate holding the tablecloth at corners
# Corner nodes (top left and top right)
corner_nodes = [nodes[0], nodes[mesh_resolution]]
for node in corner_nodes:
    constraint = fea.ChLinkPointFrame()
    constraint.Initialize(node, system.GetChFrame())
    system.Add(constraint)

# Add the mesh to the system
system.Add(mesh)

# Add a ground plane
ground_mat = chrono.ChMaterialSurfaceSMC()
ground_body = chrono.ChBodyEasyBox(2.0, 0.1, 2.0, 1000, True, True, ground_mat)
ground_body.SetPos(chrono.ChVectorD(0, -0.05, 0))
ground_body.SetBodyFixed(True)
system.Add(ground_body)

# Set up the solver
solver = mkl.ChSolverPardisoMKL()
system.SetSolver(solver)

# Set up time integration
integrator = chrono.ChTimestepperHHT(system)
integrator.SetAlpha(-0.2)
integrator.SetMaxiters(8)
integrator.SetAbsTolerances(1e-5)
integrator.SetMode(chrono.ChTimestepperHHT.POSITION)
integrator.SetScaling(True)
integrator.SetVerbose(true)
system.SetTimestepper(integrator)

# Set up visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Tablecloth Folding Simulation")
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(1.5, 1.5, 1.5), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()

# Run the simulation
time_step = 0.001
simulation_time = 5.0  # seconds

# Time integration loop
while vis.Run() and system.GetChTime() < simulation_time:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # In a real scenario, here you would apply forces to specific nodes to fold the tablecloth
    # For example, apply downward force to the free corners to simulate folding
    if system.GetChTime() > 1.0:  # Start folding after 1 second
        # Get a corner node and apply force to fold it
        fold_node = nodes[mesh_resolution * (mesh_resolution + 1)]
        fold_force = chrono.ChVectorD(0.3, -0.2, 0.3)  # Diagonal folding force
        fold_node.SetForce(fold_force)
    
    system.DoStepDynamics(time_step)
    
    # Print time
    print(f"Time: {system.GetChTime():.3f}")

print("Simulation complete!")