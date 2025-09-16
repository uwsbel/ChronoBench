import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


sys = chrono.ChSystemSMC()


contact_mat = chrono.ChMaterialSurfaceSMC()
contact_mat.SetYoungModulus(1e7)
contact_mat.SetFriction(0.3)
sys.SetDefaultMaterial(contact_mat)


length = 1.0  
width = 1.0   
num_elements_x = 20  
num_elements_y = 20  


nodes = []
for i in range(num_elements_y + 1):
    for j in range(num_elements_x + 1):
        x = j * (length / num_elements_x)
        y = i * (width / num_elements_y)
        node = chrono.ChNodeFEAxyz(chrono.ChVectorD(x, y, 0))
        sys.AddNode(node)
        nodes.append(node)


E = 1e6  
nu = 0.3  
rho = 1000  
thickness = 0.01  
mat = chrono.ChMaterialShellKirchhoff()
mat.Set_E(E)
mat.Set_nu(nu)
mat.Set_Rho(rho)


elements = []
for i in range(num_elements_y):
    for j in range(num_elements_x):
        node0 = nodes[i * (num_elements_x + 1) + j]
        node1 = nodes[i * (num_elements_x + 1) + j + 1]
        node2 = nodes[(i + 1) * (num_elements_x + 1) + j + 1]
        node3 = nodes[(i + 1) * (num_elements_x + 1) + j]
        element = chrono.ChElementShellANCF_3423()
        element.SetNodes(node0, node1, node2, node3)
        element.SetMaterial(mat)
        element.SetThickness(thickness)
        sys.AddElement(element)
        elements.append(element)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Tablecloth Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.5, 0.5, 1.0))
vis.AddTypicalLights()


sys.SetSolverType(chrono.ChSolver.Type_PARDISO_MKL)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)