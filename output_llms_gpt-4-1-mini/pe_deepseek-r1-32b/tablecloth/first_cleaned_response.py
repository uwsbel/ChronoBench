import math
import numpy as np
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


sys = chrono.ChSystemSMC()
sys.SetSolverType(chrono.ChSolver.Type_PARDISO_MKL)  


mesh = fea.ChMesh()


cloth_length = 2.0  
cloth_width = 2.0   
num_elements_x = 20
num_elements_y = 20
thickness = 0.01    


young_modulus = 1e6  
poisson_ratio = 0.3
density = 1000.0     


nodes = []
dx = cloth_length / num_elements_x
dy = cloth_width / num_elements_y

for i in range(num_elements_x + 1):
    for j in range(num_elements_y + 1):
        x = -cloth_length/2 + i * dx
        y = -cloth_width/2 + j * dy
        z = 0.0
        node = fea.ChNodeFEAxyzrot()
        node.SetPos(chrono.ChVectorD(x, y, z))
        mesh.AddNode(node)
        nodes.append(node)


for i in range(num_elements_x):
    for j in range(num_elements_y):
        node1 = nodes[i * (num_elements_y + 1) + j]
        node2 = nodes[i * (num_elements_y + 1) + j + 1]
        node3 = nodes[(i + 1) * (num_elements_y + 1) + j + 1]
        node4 = nodes[(i + 1) * (num_elements_y + 1) + j]
        
        element = fea.ChElementShellKirchhoff()
        element.SetNodes(node1, node2, node3, node4)
        
        
        material = fea.ChMaterialShellKirchhoff()
        material.SetYoungModulus(young_modulus)
        material.SetPoissonRatio(poisson_ratio)
        material.SetThickness(thickness)
        material.SetDensity(density)
        element.SetMaterial(material)
        
        mesh.AddElement(element)


sys.Add(mesh)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Tablecloth Folding Simulation")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3D(0, 3, 6))
vis.AddTypicalLights()


for i in range(num_elements_y + 1):
    node = nodes[i]
    node.SetBodyFixed(True)


gravity = chrono.ChVectorD(0, -9.81, 0)
for node in nodes:
    if not node.IsFixed():
        node.GetMass().SetGravity(gravity)


time_step = 0.001
simulation_time = 0.0


while vis.Run():
    vis.BeginScene()
    sys.DoStepDynamics(time_step)
    vis.Render()
    vis.EndScene()
    simulation_time += time_step