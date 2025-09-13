import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea

# 1. Initialize the Chrono system
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity

# 2. Create the table (rigid body)
table = chrono.ChBodyEasyBox(2.2, 0.1, 2.2, 1000)  # Dimensions: 2.2x0.1x2.2
table.SetPos(chrono.ChVectorD(0, -0.05, 0))  # Position below the cloth
table.SetBodyFixed(True)  # Static table
system.Add(table)

# 3. Create a mesh for the tablecloth
mesh = fea.ChMesh()
system.Add(mesh)

# 4. Define material properties for Kirchhoff shells
rho = 1000    # Density (kg/m³)
E = 2e7       # Young's modulus (Pa)
nu = 0.3      # Poisson's ratio
thickness = 0.005  # Cloth thickness
material = fea.ChMaterialShellKirchhoff(rho, E, nu)

# 5. Generate grid of nodes
length_x = 2.0  # Cloth dimensions (m)
length_z = 2.0
num_div_x = 15  # Mesh divisions
num_div_z = 15

# Node spacing
dx = length_x / num_div_x
dz = length_z / num_div_z

# Create nodes
nodes = []
for iz in range(num_div_z + 1):
    for ix in range(num_div_x + 1):
        # Node position (centered at origin, raised above table)
        x = ix * dx - length_x / 2
        z = iz * dz - length_z / 2
        y = 0.1  # Initial height
        node = fea.ChNodeFEAxyz(chrono.ChVectorD(x, y, z))
        nodes.append(node)
        mesh.AddNode(node)

# 6. Create quadrilateral shell elements
for iz in range(num_div_z):
    for ix in range(num_div_x):
        # Node indices for current element
        n0 = iz * (num_div_x + 1) + ix
        n1 = iz * (num_div_x + 1) + ix + 1
        n2 = (iz + 1) * (num_div_x + 1) + ix + 1
        n3 = (iz + 1) * (num_div_x + 1) + ix

        # Create element and set material/thickness
        element = fea.ChElementShellReissner4()
        element.SetNodes(nodes[n0], nodes[n1], nodes[n2], nodes[n3])
        element.AddLayer(thickness, 0, material)  # Single material layer
        mesh.AddElement(element)

# 7. Add visualization assets
vis_mesh = fea.ChVisualizationFEAmesh(mesh)
vis_mesh.SetFEMdataType(fea.FEADataType_NODE_SPEED_NORM)  # Color by velocity
vis_mesh.SetColorscaleMinMax(0, 5)  # Velocity range
vis_mesh.SetSmoothFaces(True)
mesh.AddVisualizationFEA(vis_mesh)

# 8. Set up contact surface for cloth-table interaction
contact_material = chrono.ChMaterialSurfaceSMC()
contact_surface = fea.ChContactSurfaceMesh(mesh, contact_material)
mesh.AddContactSurface(contact_surface)
contact_surface.AddFacesFromBoundary(0.001)  # Auto-detect boundary faces

# 9. Configure solver (PardisoMKL)
solver = chrono.ChSolverPardisoMKL()
system.SetSolver(solver)
system.SetSolverMaxIterations(100)
system.SetSolverForceTolerance(1e-10)

# 10. Set up Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Tablecloth Folding Simulation")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 1, -3), chrono.ChVectorD(0, 0, 0))  # Position and target
vis.AddTypicalLights()

# 11. Simulation loop
time_step = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)