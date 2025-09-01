import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.mkb as mkb


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))


sys = chrono.ChSystemSMC()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Tablecloth Folding Simulation")
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()


mesh = chrono.ChMesh()


E = 1e6  
nu = 0.3  
k = E / (3 * (1 - 2 * nu))  
mu = E / (2 * (1 + nu))  


material = chrono.ChMaterialShellKirchhoff()
material.SetYoungModulus(E)
material.SetPoissonRatio(nu)
material.SetThickness(0.01)


num_nodes_x = 10
num_nodes_y = 10
node_spacing = 0.1
for i in range(num_nodes_x):
    for j in range(num_nodes_y):
        node = chrono.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(i * node_spacing, j * node_spacing, 0)))
        mesh.AddNode(node)

for i in range(num_nodes_x - 1):
    for j in range(num_nodes_y - 1):
        node1 = mesh.GetNode(i * num_nodes_y + j)
        node2 = mesh.GetNode((i + 1) * num_nodes_y + j)
        node3 = mesh.GetNode((i + 1) * num_nodes_y + j + 1)
        node4 = mesh.GetNode(i * num_nodes_y + j + 1)
        element = chrono.ChElementShellKirchhoff()
        element.SetNodes(node1, node2, node3, node4)
        element.SetMaterial(material)
        mesh.AddElement(element)


sys.Add(mesh)


solver = chrono.ChSolverPardisoMKL()
sys.SetSolver(solver)


time_step = 0.01
total_time = 10


while sys.GetChTime() < total_time:
    sys.DoStepDynamics(time_step)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    vis.Run()


vis.Finalize()