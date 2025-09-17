import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.mesh as chronomesh
import numpy as np


sys = chrono.ChSystemNSC()


chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
chrono.ChCollisionModel.SetDefaultSuggestedRadius(0.001)


mat = chrono.ChMaterialSurfaceNSC()
mat.SetFriction(0.4)
mat.SetDampingF(0.2)
mat.SetCompliance(0.0001)
mat.SetComplianceT(0.0001)


kirchhoff_mat = chrono.ChMaterialShellKirchhoff()
kirchhoff_mat.SetDensity(1000)  
kirchhoff_mat.SetYoungModulus(1e6)  
kirchhoff_mat.SetPoissonRatio(0.3)  
kirchhoff_mat.SetThickness(0.01)  


mesh = chronomesh.ChMesh()
node_list = []
element_list = []


length = 2.0
width = 1.5
num_nodes_x = 20
num_nodes_y = 15


for i in range(num_nodes_x):
    for j in range(num_nodes_y):
        x = -length / 2 + i * length / (num_nodes_x - 1)
        y = -width / 2 + j * width / (num_nodes_y - 1)
        node = chrono.ChNodeFEAxyz(chrono.ChVectorD(x, y, 0))
        node_list.append(node)
        sys.Add(node)


for i in range(num_nodes_x - 1):
    for j in range(num_nodes_y - 1):
        node1 = node_list[i * num_nodes_y + j]
        node2 = node_list[i * num_nodes_y + j + 1]
        node3 = node_list[(i + 1) * num_nodes_y + j + 1]
        node4 = node_list[(i + 1) * num_nodes_y + j]
        element = chrono.ChElementShellANCF(node1, node2, node3, node4, kirchhoff_mat)
        element_list.append(element)
        sys.Add(element)


force = chrono.ChForce()
force.SetF0(chrono.ChVectorD(0, -10, 0))
sys.AddForce(force)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 0, 5))
vis.AddTypicalLights()


sys.SetSolver(chrono.ChSolverPardisoMKL())


time_step = 1e-3
total_time = 10


while sys.GetChTime() < total_time:
    sys.DoStepDynamics(time_step)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()


sys.Delete()