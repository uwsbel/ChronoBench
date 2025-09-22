import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl
import numpy as np


sys = chrono.ChSystemSMC()


sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


E = 1e7  
nu = 0.3  
rho = 1000  


material = chrono.ChMaterialShellKirchhoff()
material.Set_E(E)
material.Set_nu(nu)
material.Set_rho(rho)


length = 1.0
width = 1.0
num_elements_x = 20
num_elements_y = 20
dx = length / num_elements_x
dy = width / num_elements_y


mesh = chrono.ChMesh()


for i in range(num_elements_y + 1):
    for j in range(num_elements_x + 1):
        x = j * dx - length / 2
        y = 0.5  
        z = i * dy - width / 2
        mesh.AddNode(chrono.ChNodeFEAxyz(chrono.ChVectorD(x, y, z)))

for i in range(num_elements_y):
    for j in range(num_elements_x):
        node0 = i * (num_elements_x + 1) + j
        node1 = node0 + 1
        node2 = (i + 1) * (num_elements_x + 1) + j + 1
        node3 = node2 - 1
        mesh.AddElement(chrono.ChElementShellANCF_3423(material, node0, node1, node2, node3))


mesh.SetAutomaticGravity(False)  


sys.Add(mesh)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Tablecloth Simulation')
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1, -3))
vis.AddTypicalLights()


sys.SetSolverType(chrono.ChSolver.Type_PARDISO_MKL)
solver = mkl.ChSolverPardisoMKL()
sys.SetSolver(solver)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)