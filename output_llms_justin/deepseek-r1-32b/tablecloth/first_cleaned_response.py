importchrono
import math
import numpy as np


system = chrono.ChSystemNSC()
system.SetSolverType(chrono.ChSolverType.PARDISO_MKL)
system.SetSolverIterations(10)
system.SetSolverTolerance(1e-6)


vis = chrono.ChIrrApp(system, "Tablecloth Folding Simulation", chrono.CVT_UTF8, 800, 600, False)
vis.SetCamera(chrono.ChVectorD(0, 1.5, 1.5), chrono.ChVectorD(0, 0, 0))
vis.SetLight(chrono.ChVectorD(0, 1, 0), chrono.ChVectorD(0.5, 0.5, 0.5))


cloth_size = 1.0  
num_elements_x = 10  
num_elements_z = 10  
element_length = cloth_size / num_elements_x
material = chrono.ChMaterialSurface()
material.SetYoungModulus(1e6)  
material.SetPoissonRatio(0.3)  
density = 100  
thickness = 0.01  


cloth = chrono.ChMesh()
cloth.SetMaterial(material)
cloth.SetYoungModulus(1e6)
cloth.SetPoissonRatio(0.3)
cloth.SetDensity(density)
cloth.SetThickness(thickness)


nodes = []
for x in np.linspace(-cloth_size/2, cloth_size/2, num_elements_x+1):
    for z in np.linspace(-cloth_size/2, cloth_size/2, num_elements_z+1):
        y = 0.0  
        pos = chrono.ChVectorD(x, y, z)
        node = chrono.ChNodeFEAxyzD()
        node.SetPos(pos)
        system.AddNode(node)
        nodes.append(node)


for i in range(num_elements_x):
    for j in range(num_elements_z):
        
        n1 = nodes[i*(num_elements_z+1) + j]
        n2 = nodes[(i+1)*(num_elements_z+1) + j]
        n3 = nodes[(i+1)*(num_elements_z+1) + j+1]
        n4 = nodes[i*(num_elements_z+1) + j+1]
        
        
        element = chrono.ChElementShellANCF()
        element.SetNodes(n1, n2, n3, n4)
        element.SetMaterial(material)
        element.SetThickness(thickness)
        element.SetDensity(density)
        element.SetYoungModulus(1e6)
        element.SetPoissonRatio(0.3)
        element.SetMassMatrixType(chrono.ChElementShellANCF.MASSMATRIX.diag)
        element.SetStiffnessMatrixType(chrono.ChElementShellANCF.STIFFNESSMATRIX.full)
        system.AddElement(element)


for i in [0, num_elements_x]:
    for j in [0, num_elements_z]:
        node_idx = i*(num_elements_z+1) + j
        node = nodes[node_idx]
        constraint = chrono.ChLinkPointFix()
        constraint.Initialize(node, chrono.ChFrameD())
        system.Add(constraint)


light = chrono.ChLight()
light.SetDirection(chrono.ChVectorD(0, 1, 0))
light.SetIntensity(1)
vis.AddLight(light)


vis.Add(cloth)


vis.Begin()
while vis.Run():
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    chrono.ChIrrApp.Synch()
    system.DoStepDynamics(0.001)
    print("Time =", system.GetChTime())
vis.End()