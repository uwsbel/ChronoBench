import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.mesh as chronomesh
import numpy as np


sys = chrono.ChSystemNSC()


sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
ground.SetBodyFixed(True)


nx, ny = 20, 20  
dx, dy = 0.1, 0.1  
nodes = []
for i in range(nx + 1):
    for j in range(ny + 1):
        node = chrono.ChNodeFEAxyz()
        node.SetId(len(nodes))
        node.SetPos(chrono.ChVectorD(i * dx, j * dy, 0))
        sys.Add(node)
        nodes.append(node)

elements = []
for i in range(nx):
    for j in range(ny):
        node1 = nodes[i * (ny + 1) + j]
        node2 = nodes[i * (ny + 1) + j + 1]
        node3 = nodes[(i + 1) * (ny + 1) + j + 1]
        node4 = nodes[(i + 1) * (ny + 1) + j]
        element = chrono.ChElementShellKirchhoff()
        element.SetNodes(node1, node2, node3, node4)
        sys.Add(element)
        elements.append(element)


mat = chrono.ChMaterialShellKirchhoff()
mat.SetYoungModulus(1e6)  
mat.SetPoissonRatio(0.3)  
mat.SetThickness(0.01)  
for element in elements:
    element.SetMaterial(mat)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 0, 5))
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Tablecloth folding simulation")


solver = chrono.ChSolverPardisoMKL()
sys.SetSolver(solver)


dt = 1e-3  
t_end = 10  


while sys.GetChTime() < t_end:
    sys.DoStep(dt)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    vis.Run()


sys.Delete()
vis.Delete()