import pychrono

import pychrono.irrlicht

import pychrono.pardiso

import numpy as np




chrono = pychrono.ChLink_DenDenovo()

chrono.Init()




tablecloth = pychrono.ChMaterialSurfaceIsotropicKirchhoff(1.0, 1.0e5, 0.3)




num_nodes = 100

mesh_size = 10

nodes = np.zeros((num_nodes, 3))

for i in range(num_nodes):

    nodes[i, 0] = i % mesh_size * mesh_size

    nodes[i, 1] = i // mesh_size * mesh_size

    nodes[i, 2] = 0.0


elements = []

for i in range(num_nodes - 1):

    elements.append(pychrono.ChElementShellIsotropicKirchhoff(tablecloth, 0.1))




irr = pychrono.irrlicht.ChIrrApp("TableclothFolding", 800, 600)




visualizer = pychrono.irrlicht.ChVisualizer()




solver = pychrono.pardiso.ChSolverPardisoMKL()




tablecloth_body = pychrono.ChBody(chrono)

tablecloth_body.SetName("Tablecloth")

tablecloth_body.SetMass(1.0)

tablecloth_body.SetStaticForce(chrono.ChVector(0, 0, -1000))

tablecloth_body.AddAsset(visualizer)




for i, node in enumerate(nodes):

    tablecloth_body.AddNode(node)

    tablecloth_body.AddAsset(elements[i])




visualizer.SetModel(tablecloth_body)




chrono.SetSolver(solver)




chrono.DoStep(1.0)