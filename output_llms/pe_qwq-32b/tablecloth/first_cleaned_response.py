import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import numpy as np


system = chrono.ChSystemSMC()
system.SetSolverType(chrono.ChSolver.Type_PARDISO_MKL)
system.SetSolverMaxIterations(100)
system.SetTolerance(1e-6)
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


tablecloth_mesh = fea.ChMesh()
tablecloth_mesh.SetAutomaticGravity(True)


width = 2.0        
length = 2.0
nx = 20            
ny = 20
dx = width / nx
dy = length / ny


nodes = []
for i in range(nx + 1):
    row = []
    for j in range(ny + 1):
        x = -width/2 + i * dx
        y = -length/2 + j * dy
        node = fea.ChNodeFEAxyzrot()
        node.SetPos(chrono.ChVectorD(x, y, 0.0))
        tablecloth_mesh.AddNode(node)
        row.append(node)
    nodes.append(row)


young_modulus = 1e6  
poisson_ratio = 0.3
thickness = 0.01     

for i in range(nx):
    for j in range(ny):
        
        n0 = nodes[i][j]
        n1 = nodes[i+1][j]
        n2 = nodes[i+1][j+1]
        n3 = nodes[i][j+1]
        
        element = fea.ChElementShellANCF()
        element.SetNodes(n0, n1, n2, n3)
        
        material = fea.ChMaterialShellANCF()
        material.SetYoungModulus(young_modulus)
        material.SetNu(poisson_ratio)
        material.SetThickness(thickness)
        element.SetMaterial(material)
        
        tablecloth_mesh.AddElement(element)


corners = [
    nodes[0][0],     
    nodes[0][ny],    
    nodes[nx][0],    
    nodes[nx][ny]    
]
for node in corners:
    node.SetFixed(True)

system.Add(tablecloth_mesh)


visualizer = chronoirr.ChVisualSystemIrrlicht()
visualizer.SetWindowSize(1024, 768)
visualizer.SetWindowTitle("Tablecloth Folding Simulation")
visualizer.Initialize()
visualizer.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
visualizer.AddSkyBox()
visualizer.AddCamera(chronoirr.ChVectorD(0, 3, 5))
visualizer.AddTypicalLights()
visualizer.AttachSystem(system)


while visualizer.Run():
    visualizer.BeginScene()
    visualizer.Render()
    visualizer.EndScene()
    system.DoStepDynamics(0.005)