import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath("data/")
my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
my_system.SetSolverType(chrono.ChSolver.Type_PARADISO_MKL)



material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.3)
material.SetRestitution(0.2)


width = 2.0
length = 3.0
num_nodes_x = 20
num_nodes_y = 30


nodes = []
for i in range(num_nodes_x):
    for j in range(num_nodes_y):
        x = i * width / (num_nodes_x - 1)
        y = j * length / (num_nodes_y - 1)
        nodes.append(chrono.ChNodeFEAxyz(chrono.ChVectorD(x, y, 0)))


elements = []
for i in range(num_nodes_x - 1):
    for j in range(num_nodes_y - 1):
        node1 = i * num_nodes_y + j
        node2 = (i + 1) * num_nodes_y + j
        node3 = (i + 1) * num_nodes_y + (j + 1)
        node4 = i * num_nodes_y + (j + 1)
        elements.append(chrono.ChElementShellANCF(nodes[node1], nodes[node2], nodes[node3], nodes[node4]))


mesh = chrono.ChMesh()
for node in nodes:
    mesh.AddNode(node)
for element in elements:
    mesh.AddElement(element)


body = chrono.ChBody()
body.SetMesh(mesh)
body.SetBodyFixed(False)
body.AddMaterial(material)
my_system.Add(body)





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Tablecloth Folding Simulation")
vis.Initialize()
vis.Run()


while vis.Run():
    my_system.DoStepDynamics(0.01)
    vis.Render()