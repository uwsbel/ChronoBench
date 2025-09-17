import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


width = 2.0
length = 2.0
n_x = 20  
n_y = 20  


nodes = []
for i in range(n_x):
    for j in range(n_y):
        x = -width/2 + i * (width / (n_x - 1))
        y = -length/2 + j * (length / (n_y - 1))
        nodes.append(chrono.ChVectorD(x, y, 0))


elements = []
for i in range(n_x - 1):
    for j in range(n_y - 1):
        node_index_1 = i * n_y + j
        node_index_2 = (i + 1) * n_y + j
        node_index_3 = (i + 1) * n_y + (j + 1)
        node_index_4 = i * n_y + (j + 1)
        elements.append((node_index_1, node_index_2, node_index_3, node_index_4))


shell = chrono.ChShell()
shell.Set_Nodes(nodes)
shell.Set_Elements(elements)


youngs_modulus = 1e7
poissons_ratio = 0.3
thickness = 0.001


kirchhoff_mat = chrono.ChMaterialSurfaceNSC()
kirchhoff_mat.Set_Young(youngs_modulus)
kirchhoff_mat.Set_Poisson(poissons_ratio)
kirchhoff_mat.Set_Thickness(thickness)
shell.Set_Material(kirchhoff_mat)


system.Add(shell)


for i in range(n_x):
    
    node_index = i * n_y
    shell.Set_Node_Fixed(node_index, True)
    
    node_index = (i + 1) * n_y - 1
    shell.Set_Node_Fixed(node_index, True)

for j in range(n_y):
    
    node_index = j * n_x
    shell.Set_Node_Fixed(node_index, True)
    
    node_index = (n_y - 1) * n_x + j
    shell.Set_Node_Fixed(node_index, True)


system.Set_Solver_Type(chrono.ChSolver.Type_PARADISO_MKL)
system.Set_Solver_Max_Iterations(100)
system.Set_Solver_Tolerance(1e-6)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Tablecloth Folding Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 1, -2))
vis.AddTypicalLights()


time_step = 0.001
simulation_time = 5.0

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)

    if system.GetChTime() > simulation_time:
        break