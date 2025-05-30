import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


sys = chrono.ChSystemNSC()


contact_material = chrono.ChMaterialSurfaceNSC()
contact_material.SetFriction(0.5)
contact_material.SetDampingF(0.5)
contact_material.SetCompliance(1e-6)
contact_material.SetComplianceT(1e-6)


ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, contact_material)
ground.SetPos(chrono.ChVectorD(0, -2, 0))
ground.SetBodyFixed(True)


sys.Add(ground)


nx = 20
ny = 20
Lx = 5
Ly = 5
dx = Lx / (nx - 1)
dy = Ly / (ny - 1)


nodes = []
for i in range(nx):
    for j in range(ny):
        node = chrono.ChNodeFEAxyzrot()
        node.SetPos(chrono.ChVectorD(i * dx, 0, j * dy))
        node.SetMass(1)
        nodes.append(node)
        sys.Add(node)


elements = []
for i in range(nx - 1):
    for j in range(ny - 1):
        node1 = nodes[i * ny + j]
        node2 = nodes[i * ny + j + 1]
        node3 = nodes[(i + 1) * ny + j + 1]
        node4 = nodes[(i + 1) * ny + j]
        
        element = chrono.ChElementShellANCF()
        element.AddNode(node1)
        element.AddNode(node2)
        element.AddNode(node3)
        element.AddNode(node4)
        
        
        element.SetThickness(0.1)
        
        
        material = chrono.ChMaterialShellANCF()
        material.SetDensity(1000)
        material.SetYoungModulus(1e6)
        material.SetPoissonRatio(0.3)
        material.SetThickness(0.1)
        
        element.SetMaterial(material)
        
        elements.append(element)
        sys.Add(element)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()


for node in nodes:
    vis.AddPoint(node, 0.1, chrono.ChVectorD(1, 0, 0))

for element in elements:
    vis.AddShell(element, 0.1, chrono.ChVectorD(1, 0, 0))


vis.AddCamera(chrono.ChVectorD(0, 5, 5), chrono.ChVectorD(0, 0, 0))


solver = chrono.ChSolverPardisoMKL()
sys.SetSolver(solver)


while vis.Run():
    sys.DoStepDynamics(0.01)