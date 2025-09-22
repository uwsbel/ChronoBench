import chrono as ch
import chrono.irrlicht as irr
import numpy as np


ch.CH_initSystem()


system = ch.ChSystemSMC()


system.SetSolverType(ch.ChSolver.PARDISO_MKL)
system.SetSolverVerbose(True)


mat = ch.ChMaterialShellKirchhoff()
mat.SetThickness(0.001)  
mat.SetYoungModulus(100000.0)  
mat.SetPoissonRatio(0.3)  
mat.SetDensity(1000)  



nx, ny = 20, 20
dx, dy = 0.05, 0.05


mesh = ch.ChMesh()


for i in range(nx + 1):
    for j in range(ny + 1):
        x = i * dx - (nx * dx) / 2
        y = j * dy - (ny * dy) / 2
        z = 0.5
        node = ch.ChNodeFEAxyz(x, y, z)
        mesh.AddNode(node)


for i in range(nx):
    for j in range(ny):
        n1 = mesh.GetNodeByIndex(i * (ny + 1) + j)
        n2 = mesh.GetNodeByIndex((i + 1) * (ny + 1) + j)
        n3 = mesh.GetNodeByIndex((i + 1) * (ny + 1) + (j + 1))
        n4 = mesh.GetNodeByIndex(i * (ny + 1) + (j + 1))
        elem = ch.ChElementShellKirchhoff(n1, n2, n3, n4, mat)
        mesh.AddElement(elem)


system.Add(mesh)


viz = irr.ChIrrApp(system, 'Folding Tablecloth Simulation', 800, 600)
viz.SetCameraVerticalAxis()
viz.SetCameraPosition(0, -1, 0.5)
viz.EnableCollisionVisualization(True)


while viz.Run():
    system.DoStepDynamics(0.01)

    
    
    for i in range(ny + 1):
        node = mesh.GetNodeByIndex(i)
        node.SetPos_dt(ch.ChVectorD(0, 0, 0))
        node.SetRot_dt(ch.ChQuaternionD().RotateY(0.01))

viz.End()