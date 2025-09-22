import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  



E = 1e6  
nu = 0.3  
thickness = 0.001  
density = 1000  


material = chrono.ChContinuumElasticityKirchhoffIsotropic(E, nu, density)


nx = 20  
ny = 20  
width = 1.0  
height = 1.0  


mesh = chrono.fea.ChMesh()
mesh.SetAutomaticGravity(False)  


nodes = []
for i in range(nx + 1):
    for j in range(ny + 1):
        x = i * width / nx - width/2
        y = 0.5  
        z = j * height / ny - height/2
        node = chrono.fea.ChNodeFEAxyz(chrono.ChVectorD(x, y, z))
        node.SetMass(0)  
        mesh.AddNode(node)
        nodes.append(node)


for i in range(nx):
    for j in range(ny):
        
        n1 = nodes[i * (ny + 1) + j]
        n2 = nodes[(i + 1) * (ny + 1) + j]
        n3 = nodes[(i + 1) * (ny + 1) + j + 1]
        n4 = nodes[i * (ny + 1) + j + 1]

        
        element = chrono.fea.ChElementShellANCF()
        element.SetNodes(n1, n2, n3, n4)
        element.SetDimensions(width/nx, height/ny)
        element.SetMaterial(material)
        element.SetThickness(thickness)
        mesh.AddElement(element)


system.Add(mesh)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Tablecloth Folding Simulation")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1.5, -2))
vis.AddTypicalLights()
vis.AddLight(chrono.ChVectorD(0, 1.5, 0), chrono.ChVectorD(0, -1, 0), 100, 1.0, chrono.ChColor(1, 1, 1))


node_asset = chrono.fea.ChVisualizationFEAmesh(mesh)
node_asset.SetFEMdataType(chrono.fea.ChVisualizationFEAmesh.E_PLOT_NODES)
node_asset.SetSymbolsThickness(0.01)
mesh.AddAsset(node_asset)

elem_asset = chrono.fea.ChVisualizationFEAmesh(mesh)
elem_asset.SetFEMdataType(chrono.fea.ChVisualizationFEAmesh.E_PLOT_ELEMS)
elem_asset.SetWireframe(True)
mesh.AddAsset(elem_asset)


solver = chrono.ChSolverPardisoMKL()
system.SetSolver(solver)


time_step = 0.001
end_time = 5.0
output_freq = 10


if True:
    
    for corner in [0, nx, nx*(ny+1), (nx+1)*(ny+1)-1]:
        nodes[corner].SetFixed(True)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)

    
    time = system.GetChTime()
    if time > end_time:
        break