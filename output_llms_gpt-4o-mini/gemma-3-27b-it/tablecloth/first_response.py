import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import numpy as np

# --- Simulation Setup ---
sys = chrono.ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetSolverType(chrono.ChSolver.Type_PARADISO_MKL)

# --- Tablecloth Parameters ---
tablecloth_width = 2.0
tablecloth_length = 2.0
num_width_nodes = 20
num_length_nodes = 20
thickness = 0.001
young_modulus = 1e7
poisson_ratio = 0.3
density = 1000.0

# --- Material Properties ---
kirchhoff_material = fea.ChMaterialKirchhoff()
kirchhoff_material.SetYoungModulus(young_modulus)
kirchhoff_material.SetPoissonRatio(poisson_ratio)
kirchhoff_material.SetDensity(density)
kirchhoff_material.SetThickness(thickness)

# --- Mesh Generation ---
mesh = fea.ChMesh()

# Node Creation
nodes = []
for i in range(num_width_nodes):
    for j in range(num_length_nodes):
        x = i * (tablecloth_width / (num_width_nodes - 1))
        y = j * (tablecloth_length / (num_length_nodes - 1))
        node = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(x, y, 0)))
        mesh.AddNode(node)
        nodes.append(node)

# Element Creation (Shell Elements)
for i in range(num_width_nodes - 1):
    for j in range(num_length_nodes - 1):
        node1 = nodes[i * num_length_nodes + j]
        node2 = nodes[i * num_length_nodes + j + 1]
        node3 = nodes[(i + 1) * num_length_nodes + j + 1]
        node4 = nodes[(i + 1) * num_length_nodes + j]

        element = fea.ChElementShellQuad4()
        element.SetNodes(node1, node2, node3, node4)
        element.SetMaterial(kirchhoff_material)
        mesh.AddElement(element)

# --- Add Mesh to System ---
mesh.Initialize()
sys.Add(mesh)

# --- Boundary Conditions (Fixing edges) ---
# Fix the edges of the tablecloth to simulate it being placed on a table.
for i in range(num_width_nodes):
    node = nodes[i * num_length_nodes]
    node.SetFixed(True)

    node = nodes[i * num_length_nodes + num_length_nodes - 1]
    node.SetFixed(True)

for j in range(num_length_nodes):
    node = nodes[j]
    node.SetFixed(True)

    node = nodes[(num_width_nodes - 1) * num_length_nodes + j]
    node.SetFixed(True)

# --- Visualization Setup ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Tablecloth Folding Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, -5))
vis.AddTypicalLights()

# --- Simulation Loop ---
time_step = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(time_step)