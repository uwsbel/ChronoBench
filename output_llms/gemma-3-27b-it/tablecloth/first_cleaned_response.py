import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


solver = chrono.ChSolverPardisoMKL()
system.SetSolver(solver)


tablecloth_width = 2.0
tablecloth_length = 2.0
num_width_nodes = 20
num_length_nodes = 20
thickness = 0.001


nodes = []
elements = []

for i in range(num_width_nodes):
    for j in range(num_length_nodes):
        x = -tablecloth_width / 2 + i * (tablecloth_width / (num_width_nodes - 1))
        y = -tablecloth_length / 2 + j * (tablecloth_length / (num_length_nodes - 1))
        z = 0.0
        nodes.append(chrono.ChVectorD(x, y, z))

for i in range(num_width_nodes - 1):
    for j in range(num_length_nodes - 1):
        node1 = i * num_length_nodes + j
        node2 = i * num_length_nodes + j + 1
        node3 = (i + 1) * num_length_nodes + j + 1
        node4 = (i + 1) * num_length_nodes + j
        elements.append([node1, node2, node3, node4])


shell = chrono.ChShell()
shell.Set_Nodes(nodes)
shell.Set_Elements(elements)
shell.Set_Thickness(thickness)


youngs_modulus = 1e7
poissons_ratio = 0.3
density = 1000


material = chrono.ChMaterialSurfaceNSC()
material.Set_YoungModulus(youngs_modulus)
material.Set_PoissonRatio(poissons_ratio)
material.Set_Density(density)
shell.Set_Material(material)


system.Add(shell)


for i in range(num_width_nodes):
    node_index = i * num_length_nodes
    shell.Set_Node_Fixed(node_index, True)  
    node_index = i * num_length_nodes + num_length_nodes - 1
    shell.Set_Node_Fixed(node_index, True)  

for j in range(num_length_nodes):
    node_index = j
    shell.Set_Node_Fixed(node_index, True)  
    node_index = (num_width_nodes - 1) * num_length_nodes + j
    shell.Set_Node_Fixed(node_index, True)  


time_step = 0.001
simulation_time = 5.0


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Tablecloth Folding Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, -tablecloth_length/2, -tablecloth_width/2), chrono.ChVectorD(0, tablecloth_length/2, tablecloth_width/2))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)