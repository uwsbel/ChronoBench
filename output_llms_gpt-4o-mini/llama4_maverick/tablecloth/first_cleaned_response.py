import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl
import numpy as np


chrono.SetChronoDataPath("./chrono_data/")  


sys = chrono.ChSystemSMC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


mkl_solver = mkl.ChSolverPardisoMKL()
sys.SetSolver(mkl_solver)



length = 2.0
width = 1.5
num_elements_x = 40
num_elements_y = 30


nodes = []
for i in range(num_elements_y + 1):
    for j in range(num_elements_x + 1):
        x = j * (length / num_elements_x)
        y = 0.5  
        z = i * (width / num_elements_y)
        node = chrono.ChNodeFEAxyz(chrono.ChVectorD(x, y, z))
        sys.AddNode(node)
        nodes.append(node)



E = 1e7  
nu = 0.3  
rho = 1000  
thickness = 0.01  

mat = chrono.ChMaterialShellKirchhoff(E, nu, rho, thickness)

for i in range(num_elements_y):
    for j in range(num_elements_x):
        
        node1 = i * (num_elements_x + 1) + j
        node2 = node1 + 1
        node3 = (i + 1) * (num_elements_x + 1) + j + 1
        node4 = node3 - 1
        
        
        element = chrono.ChElementShellKirchhoff4()
        element.SetNodes(nodes[node1], nodes[node2], nodes[node3], nodes[node4])
        element.SetMaterial(mat)
        sys.AddElement(element)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Tablecloth Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1, 1, -2))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.001)