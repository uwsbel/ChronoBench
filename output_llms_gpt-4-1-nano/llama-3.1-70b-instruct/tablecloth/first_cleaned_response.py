import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.mkl as chronomkl
import numpy as np


mysim = chrono.ChSystemNSC()


mysim.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  
mysim.SetSolverType(chrono.ChSolver.Type_PARDISO_MKL)  
mysim.SetSolverMaxIterations(100)  
mysim.SetSolverTolerance(1e-6)  


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.5)  
material.SetDampingF(0.1)  


tablecloth_mesh = chrono.ChMesh()
tablecloth_mesh.SetMaterial(material)


tablecloth_length = 2.0
tablecloth_width = 1.5
tablecloth_thickness = 0.01


nodes = []
for i in range(10):
    for j in range(15):
        node = chrono.ChNodeFEAxyz()
        node.SetId(i * 15 + j)
        node.SetPos(chrono.ChVectorD(i * tablecloth_length / 9, j * tablecloth_width / 14, 0))
        tablecloth_mesh.AddNode(node)
        nodes.append(node)


elements = []
for i in range(9):
    for j in range(14):
        element = chrono.ChElementShellKirchhoffIso()
        element.SetNodes(nodes[i * 15 + j], nodes[i * 15 + j + 1], nodes[(i + 1) * 15 + j + 1], nodes[(i + 1) * 15 + j])
        element.SetThickness(tablecloth_thickness)
        tablecloth_mesh.AddElement(element)
        elements.append(element)


mysim.Add(tablecloth_mesh)


vis = chronoirr.ChVisualSystemIrrlicht(mysim)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Tablecloth Folding Simulation")
vis.SetCamera(chrono.ChVectorD(0, 0, 2), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))


while vis.Run():
    mysim.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()