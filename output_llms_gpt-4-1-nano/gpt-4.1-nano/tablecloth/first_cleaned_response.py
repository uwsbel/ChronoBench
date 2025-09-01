import pychrono.core as chrono
import pychrono.irrlicht as irr
import numpy as np


sys = chrono.ChSystemSMC()


sys.SetSolverType(chrono.ChSolverPardisoMKL())






application = irr.ChIrrApp(sys, "Tablecloth Folding Simulation", irr.dimension2d.IRON_SCREEN)
application.AddTypicalSky()
application.AddTypicalCamera(irr.vector3df(0, 1, -2))
application.AddTypicalLights()


num_nodes_x = 20  
num_nodes_y = 20  
cloth_length_x = 1.0  
cloth_length_y = 1.0  
node_spacing_x = cloth_length_x / (num_nodes_x - 1)
node_spacing_y = cloth_length_y / (num_nodes_y - 1)


nodes = []
for j in range(num_nodes_y):
    for i in range(num_nodes_x):
        x = i * node_spacing_x
        y = j * node_spacing_y
        z = 0  
        node = chrono.ChNodeFEAxyz(chrono.VNULL, chrono.ChVectorD(x, y, z))
        nodes.append(node)


mesh = chrono.ChMesh()


for node in nodes:
    mesh.AddNode(node)



E = 1e7  
nu = 0.3  
density = 1000  
thickness = 0.005  

material = chrono.ChMaterialShellKirchhoff()
material.SetYoungModulus(E)
material.SetPoissonRatio(nu)



for j in range(num_nodes_y - 1):
    for i in range(num_nodes_x - 1):
        
        n1 = i + j * num_nodes_x
        n2 = (i + 1) + j * num_nodes_x
        n3 = (i + 1) + (j + 1) * num_nodes_x
        n4 = i + (j + 1) * num_nodes_x

        
        node1 = nodes[n1]
        node2 = nodes[n2]
        node3 = nodes[n3]
        node4 = nodes[n4]

        
        element = chrono.ChElasticShellQuad4(node1, node2, node3, node4, material, thickness)
        mesh.AddShellElement(element)


sys.Add(mesh)


sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))



for i in range(num_nodes_x):
    top_node = nodes[i]
    top_node.SetFixed(True)





application.AssetBindAll()
application.AssetUpdateAll()


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    sys.DoStepDynamics(1e-4)