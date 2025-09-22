import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.mkl as chronomkl
import numpy as np


chrono.SetChronoDataPath('./data/')


mysystem = chrono.ChSystemNSC()
solver = chronomkl.ChSolverMKL()
mysystem.SetSolver(solver)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Tablecloth Folding Simulation')
vis.SetCamera(chrono.ChVectorD(0, 0, 1), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))


E = 1e6  
nu = 0.3  
rho = 1000  
thickness = 0.01  


material = chrono.ChMaterialShellKirchhoffIso(E, nu, rho, thickness)


n_x, n_y = 10, 10
length, width = 1, 1
nodes = np.zeros((n_x, n_y, 3))
for i in range(n_x):
    for j in range(n_y):
        nodes[i, j, :] = [i * length / (n_x - 1), j * width / (n_y - 1), 0]


for i in range(n_x - 1):
    for j in range(n_y - 1):
        node1 = chrono.ChNodeShell(nodes[i, j, :])
        node2 = chrono.ChNodeShell(nodes[i + 1, j, :])
        node3 = chrono.ChNodeShell(nodes[i + 1, j + 1, :])
        node4 = chrono.ChNodeShell(nodes[i, j + 1, :])
        mysystem.Add(node1)
        mysystem.Add(node2)
        mysystem.Add(node3)
        mysystem.Add(node4)
        element = chrono.ChElementShellKirchhoffIso(node1, node2, node3, node4, material)
        mysystem.Add(element)


for i in range(n_x):
    for j in range(n_y):
        node = mysystem.GetNode(i * n_y + j)
        node.SetPos(chrono.ChVectorD(nodes[i, j, :]))
        node.SetVel(chrono.ChVectorD(0, 0, 0))


center_node = mysystem.GetNode(n_x * n_y // 2)
force = chrono.ChForce()
force.SetF0(chrono.ChVectorD(0, 0, -100))
center_node.AddForce(force)


for i in range(n_x - 1):
    for j in range(n_y - 1):
        node1 = mysystem.GetNode(i * n_y + j)
        node2 = mysystem.GetNode((i + 1) * n_y + j)
        node3 = mysystem.GetNode((i + 1) * n_y + j + 1)
        node4 = mysystem.GetNode(i * n_y + j + 1)
        vis.AddTriangle(node1, node2, node3)
        vis.AddTriangle(node1, node3, node4)


stepping = chrono.ChTimestepperHHT(mysystem)
stepping.SetStep(0.01)

while vis.Run():
    mysystem.DoStepDynamics(stepping.GetStep())
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()