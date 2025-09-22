import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  




material = chrono.ChMaterialShellKirchhoff(1000,  
                                           1e6,   
                                           0.3,   
                                           1e-2)  


nx, ny = 20, 20  
width, height = 2.0, 1.5  
dx, dy = width/(nx-1), height/(ny-1)  


mesh = chrono.ChMesh()


nodes = []
for i in range(nx):
    for j in range(ny):
        x, y = i*dx - width/2, j*dy - height/2
        node = chrono.ChNodeFEMxyzD(chrono.ChVectorD(x, y, 0.0))
        node.SetMass(0.0)  
        mesh.AddNode(node)
        nodes.append(node)


for i in range(nx-1):
    for j in range(ny-1):
        
        n0 = nodes[i*ny + j]
        n1 = nodes[(i+1)*ny + j]
        n2 = nodes[(i+1)*ny + j+1]
        n3 = nodes[i*ny + j+1]

        
        element = chrono.ChElementShellANCF_3434()
        element.SetNodes(n0, n1, n2, n3)
        element.SetMaterial(material)
        mesh.AddElement(element)


system.Add(mesh)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Tablecloth Folding Simulation")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0.5, 3))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(2, 4, 2), chrono.ChVectorD(0, 0, 0), 5, 1, 10, 40, 512)


solver = chrono.ChSolverPardisoMKL()
system.SetSolver(solver)


time_step = 0.001
simulation_time = 5.0


for node in nodes:
    if node.GetPos().x > 0.5:  
        node.SetPos(chrono.ChVectorD(node.GetPos().x, node.GetPos().y + 0.1, node.GetPos().z))


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)

    
    current_time = system.GetChTime()
    print(f"Simulation time: {current_time:.2f} s")

    if current_time >= simulation_time:
        break