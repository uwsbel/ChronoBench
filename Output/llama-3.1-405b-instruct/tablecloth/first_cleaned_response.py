import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath('/path/to/chrono/data/')


sys = chrono.ChSystemNSC()


num_nodes_x = 20
num_nodes_y = 20
node_mass = 0.1
node_radius = 0.01
mesh_size_x = 1.0
mesh_size_y = 1.0

nodes = []
for i in range(num_nodes_x):
    for j in range(num_nodes_y):
        node = chrono.ChBodyEasyMesh(sys)
        node.SetPos(chrono.ChVectorD(i * mesh_size_x / (num_nodes_x - 1), j * mesh_size_y / (num_nodes_y - 1), 0))
        node.SetMass(node_mass)
        node.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
        nodes.append(node)


elements = []
for i in range(num_nodes_x - 1):
    for j in range(num_nodes_y - 1):
        node1 = nodes[i * num_nodes_y + j]
        node2 = nodes[(i + 1) * num_nodes_y + j]
        node3 = nodes[(i + 1) * num_nodes_y + j + 1]
        node4 = nodes[i * num_nodes_y + j + 1]
        element = chrono.ChElementShellKirchhoff(sys)
        element.SetNodes(node1, node2, node3, node4)
        element.SetMaterial(chrono.ChMaterialShellKirchhoff(sys))
        element.SetThickness(0.01)
        elements.append(element)


for node in nodes:
    sys.Add(node)
for element in elements:
    sys.Add(element)


application = chronoirr.ChIrrApp(sys, 'Tablecloth Folding', chronoirr.dimension2du(800, 600))
application.AddTypicalSky()
application.AddTypicalCamera(chronoirr.vector3df(0, 0, 1))
application.AddLightWithShadow(chronoirr.vector3df(2, 4, 2), chronoirr.vector3df(0, 0, 0), 3, 2, 10, 120)


solver = chrono.ChSolverPardisoMKL()
sys.SetSolver(solver)


sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
sys.SetMaxItersSolverSpeed(100)
sys.SetMaxItersSolverStab(100)


application.AssetBindAll()
application.AssetUpdateAll()
application.Simulate(0.01, 10)


application.GetDevice().closeDevice()