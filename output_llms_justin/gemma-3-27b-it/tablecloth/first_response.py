import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# --- Create a Chrono physical system ---
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity

# --- Create the tablecloth mesh ---
width = 2.0
length = 2.0
n_x = 20  # Number of nodes in x direction
n_y = 20  # Number of nodes in y direction

# Create nodes
nodes = []
for i in range(n_x):
    for j in range(n_y):
        x = -width/2 + i * (width / (n_x - 1))
        y = -length/2 + j * (length / (n_y - 1))
        nodes.append(chrono.ChVectorD(x, y, 0))

# Create elements (quads)
elements = []
for i in range(n_x - 1):
    for j in range(n_y - 1):
        node_index_1 = i * n_y + j
        node_index_2 = (i + 1) * n_y + j
        node_index_3 = (i + 1) * n_y + (j + 1)
        node_index_4 = i * n_y + (j + 1)
        elements.append((node_index_1, node_index_2, node_index_3, node_index_4))

# --- Create the shell ---
shell = chrono.ChShell()
shell.Set_Nodes(nodes)
shell.Set_Elements(elements)

# --- Material properties ---
youngs_modulus = 1e7
poissons_ratio = 0.3
thickness = 0.001

# Create an isotropic Kirchhoff material
kirchhoff_mat = chrono.ChMaterialSurfaceNSC()
kirchhoff_mat.Set_Young(youngs_modulus)
kirchhoff_mat.Set_Poisson(poissons_ratio)
kirchhoff_mat.Set_Thickness(thickness)
shell.Set_Material(kirchhoff_mat)

# --- Add the shell to the system ---
system.Add(shell)

# --- Fix the edges of the tablecloth ---
for i in range(n_x):
    # Fix left edge
    node_index = i * n_y
    shell.Set_Node_Fixed(node_index, True)
    # Fix right edge
    node_index = (i + 1) * n_y - 1
    shell.Set_Node_Fixed(node_index, True)

for j in range(n_y):
    # Fix bottom edge
    node_index = j * n_x
    shell.Set_Node_Fixed(node_index, True)
    # Fix top edge
    node_index = (n_y - 1) * n_x + j
    shell.Set_Node_Fixed(node_index, True)

# --- Solver setup ---
system.Set_Solver_Type(chrono.ChSolver.Type_PARADISO_MKL)
system.Set_Solver_Max_Iterations(100)
system.Set_Solver_Tolerance(1e-6)

# --- Visualization ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Tablecloth Folding Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 1, -2))
vis.AddTypicalLights()

# --- Simulation loop ---
time_step = 0.001
simulation_time = 5.0

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)

    if system.GetChTime() > simulation_time:
        break