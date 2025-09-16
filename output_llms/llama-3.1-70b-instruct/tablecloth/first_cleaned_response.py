import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.mkl as chronomkl
import numpy as np


chrono.SetChronoDataPath('./')


mysystem = chrono.ChSystemNSC()
mysystem.SetSolver(chronomkl.ChSolverPardisoMKL())
mysystem.SetSolverMaxIterations(100)
mysystem.SetSolverTolerance(1e-6)


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.5)
material.SetDampingF(0.1)
material.SetCompliance(1e-6)


tablecloth_mesh = chrono.ChMesh()
tablecloth_mesh.SetAutomaticGravity(False)


nx = 10
ny = 10
dx = 0.1
dy = 0.1


nodes = []
for i in range(nx):
    for j in range(ny):
        node = chrono.ChNodeFEAxyz()
        node.SetMass(1.0)
        node.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
        node.SetInertiaXY(chrono.ChVectorD(0, 0, 0))
        node.SetInertiaXZ(chrono.ChVectorD(0, 0, 0))
        node.SetInertiaYY(chrono.ChVectorD(1, 1, 1))
        node.SetInertiaYZ(chrono.ChVectorD(0, 0, 0))
        node.SetInertiaZZ(chrono.ChVectorD(1, 1, 1))
        node.SetPos(chrono.ChVectorD(i*dx, j*dy, 0))
        nodes.append(node)


for node in nodes:
    tablecloth_mesh.AddNode(node)


elements = []
for i in range(nx-1):
    for j in range(ny-1):
        node1 = nodes[i*ny + j]
        node2 = nodes[i*ny + j + 1]
        node3 = nodes[(i+1)*ny + j + 1]
        node4 = nodes[(i+1)*ny + j]
        element = chrono.ChElementShellKirchhoffIsotropic()
        element.SetNodes(node1, node2, node3, node4)
        element.SetMaterial(material)
        element.SetThickness(0.01)
        elements.append(element)


for element in elements:
    tablecloth_mesh.AddElement(element)


mysystem.Add(tablecloth_mesh)


myapplication = chronoirr.ChVisualSystemIrrlicht()
myapplication.SetWindowSize(1024, 768)
myapplication.SetWindowTitle("Tablecloth Folding Simulation")
myapplication.SetCameraDistance(2)
myapplication.SetCameraAzimuth(45)
myapplication.SetCameraElevation(45)


myapplication.AddSkyBox()
myapplication.AddLights()
myapplication.AddTypicalLigths()
myapplication.AddCamera(chrono.ChVectorD(0, 0, 1.5), chrono.ChVectorD(0, 0, 0))
myapplication.AddTypicalLights()
myapplication.AddShadow()
myapplication.AddTablecloth(tablecloth_mesh)


while myapplication.Run():
    mysystem.DoStepDynamics(0.01)
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.EndScene()