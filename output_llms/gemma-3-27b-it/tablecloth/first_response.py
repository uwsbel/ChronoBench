import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# --- Create the Chrono physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity

# --- Create the PardisoMKL solver
solver = chrono.ChSolverPardisoMKL()
system.SetSolver(solver)

# --- Tablecloth parameters
tablecloth_length = 2.0
tablecloth_width = 2.0
num_x = 20  # Number of nodes along the length
num_y = 20  # Number of nodes along the width
thickness = 0.001

# --- Create the mesh grid
nodes = []
elements = []

for i in range(num_x):
    for j in range(num_y):
        x = -tablecloth_length / 2 + i * (tablecloth_length / (num_x - 1))
        y = -tablecloth_width / 2 + j * (tablecloth_width / (num_y - 1))
        nodes.append(chrono.ChVectorD(x, y, 0))

for i in range(num_x - 1):
    for j in range(num_y - 1):
        n1 = i * num_y + j
        n2 = (i + 1) * num_y + j
        n3 = (i + 1) * num_y + (j + 1)
        n4 = i * num_y + (j + 1)
        elements.append((n1, n2, n3, n4))

# --- Create the shell
tablecloth = chrono.ChShell()
tablecloth.Set_Size(num_x, num_y)
tablecloth.Set_Thickness(thickness)

# Add nodes and elements to the shell
for i, node in enumerate(nodes):
    tablecloth.AddNode(node)

for element in elements:
    tablecloth.AddElement(*element)

# --- Material properties (isotropic Kirchhoff)
youngs_modulus = 1000.0
poissons_ratio = 0.3
density = 1.0

material = chrono.ChMaterialSurfaceNSC()
material.SetYoungModulus(youngs_modulus)
material.SetPoissonRatio(poissons_ratio)
material.SetDensity(density)
tablecloth.SetMaterial(material)

# --- Add the shell to the system
system.Add(tablecloth)

# --- Fix the corners of the tablecloth (example)
tablecloth.SetFixed(0, True)  # Fix node 0
tablecloth.SetFixed(num_y - 1, True)  # Fix node num_y - 1
tablecloth.SetFixed((num_x - 1) * num_y, True)  # Fix node (num_x - 1) * num_y
tablecloth.SetFixed((num_x - 1) * num_y + (num_y - 1), True)  # Fix node (num_x - 1) * num_y + (num_y - 1)


# --- Visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Tablecloth Folding')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 0, -2))
vis.AddTypicalLights()

# --- Simulation loop
time_step = 0.001
simulation_time = 5.0

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)

    if system.GetChTime() > simulation_time:
        break