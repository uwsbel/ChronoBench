import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# --- Simulation setup ---
chrono.SetChronoDataPath("../data/")  # Adjust path as needed

# Create a physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity

# --- Tablecloth parameters ---
tablecloth_width = 2.0
tablecloth_length = 2.0
num_x = 20  # Number of nodes along width
num_y = 20  # Number of nodes along length
thickness = 0.001

# --- Material properties ---
youngs_modulus = 1e7  # Example value
poisson_ratio = 0.3
density = 1000

# --- Create the tablecloth mesh ---
tablecloth = chrono.ChBodyEasy()
tablecloth.SetBodyFixed(False)
tablecloth.SetMass(1.0)
system.Add(tablecloth)

# Create nodes
nodes = []
for i in range(num_x):
    for j in range(num_y):
        x = -tablecloth_width / 2 + i * (tablecloth_width / (num_x - 1))
        y = -tablecloth_length / 2 + j * (tablecloth_length / (num_y - 1))
        z = 0.0
        node = chrono.ChNode(chrono.ChVectorD(x, y, z))
        nodes.append(node)
        tablecloth.AddNode(node)

# Create shell elements
for i in range(num_x - 1):
    for j in range(num_y - 1):
        node1 = nodes[i * num_y + j]
        node2 = nodes[(i + 1) * num_y + j]
        node3 = nodes[(i + 1) * num_y + j + 1]
        node4 = nodes[i * num_y + j + 1]

        element = chrono.ChShellElement()
        element.SetNodes(node1, node2, node3, node4)
        tablecloth.AddShellElement(element)

# --- Material setup ---
material = chrono.ChMaterialSurfaceNSC()
material.SetYoungModulus(youngs_modulus)
material.SetPoissonRatio(poisson_ratio)
material.SetDensity(density)
material.SetDampingFriction(0.1)  # Add some damping
tablecloth.SetMaterialSurface(material)

# --- Solver setup ---
system.SetSolverType(chrono.ChSolver.Type_PARDISO_MKL)
system.SetSolverMaxIterations(100)
system.SetSolverTolerance(1e-6)

# --- Visualization setup ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Tablecloth Folding Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 0, -2))
vis.AddTypicalLights()

# --- Simulation loop ---
time_step = 0.001
time_end = 5.0

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)

    if system.GetChTime() > time_end:
        break

print("Simulation finished.")