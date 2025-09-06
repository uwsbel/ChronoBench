import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath('PATH_TO_CHRONO_DATA')  


system = chrono.ChSystemNSC()


system.SetSolverType(chrono.ChSolver.Type_PARDISO_MKL)
system.SetSolverWarmStarting(True)


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))




density = 0.1  
young_modulus = 1e6  
poisson_ratio = 0.3
thickness = 0.001  


material = chrono.ChContinuumMaterialKirchhoff(density, young_modulus, poisson_ratio, thickness)


length_x = 1.0  
length_y = 1.0  
num_elements_x = 20
num_elements_y = 20


mesh = chrono.fea.ChMesh()


nodes = []
for i in range(num_elements_x + 1):
    for j in range(num_elements_y + 1):
        x = i * length_x / num_elements_x
        y = j * length_y / num_elements_y
        node = chrono.fea.ChNodeFEAxyz(chrono.ChVectorD(x, 0, y))
        node.SetMass(0)  
        mesh.AddNode(node)
        nodes.append(node)


elements = []
for i in range(num_elements_x):
    for j in range(num_elements_y):
        
        n0 = nodes[i * (num_elements_y + 1) + j]
        n1 = nodes[(i + 1) * (num_elements_y + 1) + j]
        n2 = nodes[(i + 1) * (num_elements_y + 1) + j + 1]
        n3 = nodes[i * (num_elements_y + 1) + j + 1]

        
        element = chrono.fea.ChElementShellANCF_3423()
        element.SetNodes(n0, n1, n2, n3)
        element.SetMaterial(material)
        mesh.AddElement(element)
        elements.append(element)


system.Add(mesh)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Tablecloth Folding Simulation")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.5, 0.5, 2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1, 3, 1), chrono.ChVectorD(0, 0, 0), 5, 1, 10, 40, 512)


node_asset = chrono.fea.ChVisualizationFEAmesh(mesh)
node_asset.SetFEMdataType(chrono.fea.ChVisualizationFEAmesh.E_PLOT_NODE_DOT_POS)
node_asset.SetSymbolsThickness(0.01)
mesh.AddAsset(node_asset)

element_asset = chrono.fea.ChVisualizationFEAmesh(mesh)
element_asset.SetFEMdataType(chrono.fea.ChVisualizationFEAmesh.E_PLOT_ELEM_SHELL_3)
element_asset.SetWireframe(True)
element_asset.SetColor(chrono.ChColor(0.7, 0.7, 0.9))
mesh.AddAsset(element_asset)


time_step = 0.001
sim_time = 5.0

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)

    
    current_time = system.GetChTime()
    if current_time >= sim_time:
        break

    
    if current_time < 0.1:
        for node in nodes:
            if node.GetPos().x > 0.4 and node.GetPos().x < 0.6 and node.GetPos().z > 0.4 and node.GetPos().z < 0.6:
                node.SetPos(node.GetPos() + chrono.ChVectorD(0, 0.01, 0))