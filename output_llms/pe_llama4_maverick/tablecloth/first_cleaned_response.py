import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import numpy as np


sys = chrono.ChSystemSMC()


sys.Set_G_acc(chrono.ChVector3d(0, -9.81, 0))


sys.SetSolverType(chrono.ChSolver.Type_PARDISO_MKL)


mesh = fea.ChMesh()


E = 1e7  
nu = 0.3  
rho = 1000  
mat = fea.ChMaterialShellKirchhoffIsothropic(E, nu, rho)


L_x = 2.0  
L_y = 1.0  
n_x = 40   
n_y = 20   


dx = L_x / n_x
dy = L_y / n_y

for i in range(n_x + 1):
    for j in range(n_y + 1):
        x = i * dx - L_x / 2
        y = j * dy
        z = 0.0
        node = fea.ChNodeFEAxyz(chrono.ChVector3d(x, y, z))
        mesh.AddNode(node)


for i in range(n_x):
    for j in range(n_y):
        node1 = mesh.GetNode(i * (n_y + 1) + j)
        node2 = mesh.GetNode((i + 1) * (n_y + 1) + j)
        node3 = mesh.GetNode((i + 1) * (n_y + 1) + (j + 1))
        node4 = mesh.GetNode(i * (n_y + 1) + (j + 1))
        
        element = fea.ChElementShellANCF_3423()
        element.SetNodes(node1, node2, node3, node4)
        element.SetMaterial(mat)
        mesh.AddElement(element)


sys.Add(mesh)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Tablecloth Folding Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1, 3))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    sys.DoStepDynamics(0.001)
    vis.EndScene()