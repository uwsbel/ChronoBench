import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# 1. Initialize PyChrono environment and core components
chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  # Set this to your Chrono data path

# Create the physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# 2. Add physical systems and objects

# Create the tablecloth material (isotropic Kirchhoff)
material = chrono.ChMaterialShellKirchhoff(1000,  # density [kg/m^3]
                                           1e6,   # Young's modulus [Pa]
                                           0.3,   # Poisson's ratio
                                           1e-2)  # thickness [m]

# Create a mesh grid for the tablecloth
nx, ny = 20, 20  # Number of elements in x and y directions
width, height = 2.0, 1.5  # Dimensions of the tablecloth [m]
dx, dy = width/(nx-1), height/(ny-1)  # Element sizes

# Create the mesh
mesh = chrono.ChMesh()

# Add nodes to the mesh
nodes = []
for i in range(nx):
    for j in range(ny):
        x, y = i*dx - width/2, j*dy - height/2
        node = chrono.ChNodeFEMxyzD(chrono.ChVectorD(x, y, 0.0))
        node.SetMass(0.0)  # Mass will be handled by elements
        mesh.AddNode(node)
        nodes.append(node)

# Add shell elements to the mesh
for i in range(nx-1):
    for j in range(ny-1):
        # Get the four nodes of the element
        n0 = nodes[i*ny + j]
        n1 = nodes[(i+1)*ny + j]
        n2 = nodes[(i+1)*ny + j+1]
        n3 = nodes[i*ny + j+1]

        # Create the shell element
        element = chrono.ChElementShellANCF_3434()
        element.SetNodes(n0, n1, n2, n3)
        element.SetMaterial(material)
        mesh.AddElement(element)

# Add the mesh to the system
system.Add(mesh)

# 3. Set up visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Tablecloth Folding Simulation")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0.5, 3))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(2, 4, 2), chrono.ChVectorD(0, 0, 0), 5, 1, 10, 40, 512)

# Set up the PardisoMKL solver
solver = chrono.ChSolverPardisoMKL()
system.SetSolver(solver)

# Simulation parameters
time_step = 0.001
simulation_time = 5.0

# Apply initial conditions (optional: add some initial deformation)
for node in nodes:
    if node.GetPos().x > 0.5:  # Example: lift one side
        node.SetPos(chrono.ChVectorD(node.GetPos().x, node.GetPos().y + 0.1, node.GetPos().z))

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)

    # Print current simulation time
    current_time = system.GetChTime()
    print(f"Simulation time: {current_time:.2f} s")

    if current_time >= simulation_time:
        break