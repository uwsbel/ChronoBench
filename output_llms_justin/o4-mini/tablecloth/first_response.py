#
# Folding tablecloth with Kirchhoff shell elements in PyChrono
#
import chrono
import chrono_irrlicht
from math import sin, cos, pi

# -----------------------------------------------------------------
# 1) Create the Chrono system and set global parameters
# -----------------------------------------------------------------
system = chrono.ChSystem()  
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Use PardisoMKL for FEA solver
system.SetSolverType(chrono.ChSolver.PARDISO_MKL)
solver = system.GetSolver()
solver.SetVerbose(False)
# Sparsity pattern won't change since mesh topology is fixed.
solver.LockSparsityPattern(True)

# -----------------------------------------------------------------
# 2) Create an FEA mesh, material, and build the "tablecloth"
# -----------------------------------------------------------------
mesh = chrono.ChMesh()

# Cloth dimensions and mesh resolution
cloth_length_x = 1.0   # along X
cloth_length_y = 1.0   # along Y
num_nodes_x   = 21
num_nodes_y   = 21
dx = cloth_length_x / (num_nodes_x - 1)
dy = cloth_length_y / (num_nodes_y - 1)

# Shell material (isotropic Kirchhoff)
thickness = 0.005
rho       = 500.0          # kg/m^3
E         = 2e7            # Young's modulus
nu        = 0.3            # Poisson ratio
mat_shell = chrono.ChMaterialShellKirchhoff(rho, E, nu, thickness)

# Create grid of ANCF shell nodes
nodes = []
for j in range(num_nodes_y):
    y = j * dy
    for i in range(num_nodes_x):
        x = i * dx
        # initial normal points +Z
        node = chrono.ChNodeFEAxyzD(chrono.ChVectorD(x, y, 0),
                                    chrono.ChVectorD(0, 0, 1))
        # Fix the nodes along the y=0 edge to simulate a held edge
        if j == 0:
            node.SetFixed(True)
        mesh.AddNode(node)
        nodes.append(node)

# Create quadrilateral ANCF shell elements
for j in range(num_nodes_y - 1):
    for i in range(num_nodes_x - 1):
        # Node indices in the flat list
        n0 = j   * num_nodes_x + i
        n1 = j   * num_nodes_x + (i+1)
        n2 = (j+1)* num_nodes_x + (i+1)
        n3 = (j+1)* num_nodes_x + i

        elm = chrono.ChElementShellANCF()
        elm.SetNodes(nodes[n0], nodes[n1], nodes[n2], nodes[n3])
        elm.SetMaterial(mat_shell)
        elm.SetAlphaDamp(0.2)    # some damping
        mesh.AddElement(elm)

# Add the mesh to the system
system.Add(mesh)

# Optional: visualize the shell mesh with wireframe
mesh_asset = chrono.ChVisualizationFEAmesh(mesh)
mesh_asset.SetFEMdataType(chrono.ChVisualizationFEAmesh.E_PLOT_NODE_SPEED)
mesh_asset.SetColorscaleMinMax(0.0, 2.0)
mesh_asset.SetSmoothFaces(True)
mesh_asset.SetWireframe(True)
mesh.AddAsset(mesh_asset)

# -----------------------------------------------------------------
# 3) Set up the Irrlicht visualization
# -----------------------------------------------------------------
app = chrono_irrlicht.ChIrrApp(system, "Tablecloth Folding Demo",
                                chrono_irrlicht.dimension2du(1024, 768))
app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalLights()
app.AddTypicalCamera(chrono.ChVectorD(1.2, 0.5, 1.2),
                     chrono.ChVectorD(0.5, 0.5, 0.0))

app.AssetBindAll()
app.AssetUpdateAll()

# -----------------------------------------------------------------
# 4) Simulation loop
# -----------------------------------------------------------------
app.SetTimestep(1e-3)
while app.Run():
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    system.DoStepDynamics(1e-3)