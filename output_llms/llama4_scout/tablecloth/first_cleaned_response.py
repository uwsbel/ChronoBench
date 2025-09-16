import chrono as ch
import chrono.irrlicht as irr
import numpy as np


ch.ChEngine.Initialize()


system = ch.ChSystem_EulerImplicit()


system.SetSolverType(ch.ChSolver.PARDISO_MKL)
system.SetSolverVerbose(True)


mat = ch.ChMaterialShellKirchhoff()
mat.SetThickness(0.001)  
mat.SetYoungModulus(100e3)  
mat.SetPoissonRatio(0.3)  
mat.SetDensity(1000)  



nx, ny = 20, 20
Lx, Ly = 1.0, 1.0  


mesh = ch.ChMesh()


for i in range(nx + 1):
    x = Lx * i / nx
    for j in range(ny + 1):
        y = Ly * j / ny
        node = ch.ChNodeFEAxyzP(x, y, 0)
        node.SetMass(0)  
        mesh.AddNode(node)


for i in range(nx):
    for j in range(ny):
        n1 = mesh.GetNodeByIndex(i * (ny + 1) + j)
        n2 = mesh.GetNodeByIndex((i + 1) * (ny + 1) + j)
        n3 = mesh.GetNodeByIndex((i + 1) * (ny + 1) + (j + 1))
        n4 = mesh.GetNodeByIndex(i * (ny + 1) + (j + 1))
        
        
        elem = ch.ChElementShellQuadrilateral()
        elem.SetNodes(n1, n2, n3, n4)
        elem.SetMaterial(mat)
        mesh.AddElement(elem)


system.Add(mesh)


viz = irr.ChIrrApp(system, 'Folding Tablecloth Simulation', 800, 600)
viz.AttachSystem()


viz.Run()


ch.ChEngine.Finalize()