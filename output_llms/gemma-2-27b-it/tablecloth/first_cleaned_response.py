import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath("data/")
my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))



length = 2.0
width = 1.5


num_nodes_x = 20
num_nodes_y = 15
nodes = np.zeros((num_nodes_x * num_nodes_y, 3))
for i in range(num_nodes_x):
    for j in range(num_nodes_y):
        nodes[i * num_nodes_y + j, :] = [i * length / (num_nodes_x - 1), j * width / (num_nodes_y - 1), 0]


elements = []
for i in range(num_nodes_x - 1):
    for j in range(num_nodes_y - 1):
        elements.append([i * num_nodes_y + j, (i + 1) * num_nodes_y + j, (i + 1) * num_nodes_y + j + 1, i * num_nodes_y + j + 1])


tablecloth = chrono.ChShellMesh()
tablecloth.SetNodes(nodes)
tablecloth.SetElements(elements)


material = chrono.ChMaterialShellKirchhoff()
material.SetYoungModulus(1000)  
material.SetPoissonRatio(0.3)
material.SetDensity(1000)  
tablecloth.SetMaterial(material)


my_system.Add(tablecloth)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Tablecloth Folding Simulation")
vis.Initialize()


solver = chrono.ChSolverPardisoMKL()
my_system.SetSolver(solver)


while vis.Run():
    my_system.DoStepDynamics(0.01)
    vis.Render()