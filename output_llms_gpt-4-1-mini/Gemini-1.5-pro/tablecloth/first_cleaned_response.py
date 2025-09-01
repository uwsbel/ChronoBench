import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr
import numpy as np






time_step = 1e-3


total_time = 5


output_frequency = 100


gravity = chrono.ChVectorD(0, -9.81, 0)


visualization_scaling = 1





system = chrono.ChSystemSMC()
system.Set_G_acc(gravity)


solver = chrono.ChSolverPARDISOMKL()
system.SetSolver(solver)






rho = 1000  
E = 1e7  
nu = 0.3  
thickness = 0.005  


width = 1
length = 1


nx = 20
ny = 20


mesh = fea.ChMesh()


nodes = {}
for i in range(nx + 1):
    for j in range(ny + 1):
        x = i * (width / nx)
        y = j * (length / ny)
        node = fea.ChNodeShellXYZ(chrono.ChVectorD(x, 0.5, y), chrono.ChVectorD(0, 1, 0))
        nodes[(i, j)] = node
        mesh.AddNode(node)


for i in range(nx):
    for j in range(ny):
        node1 = nodes[(i, j)]
        node2 = nodes[(i + 1, j)]
        node3 = nodes[(i + 1, j + 1)]
        node4 = nodes[(i, j + 1)]
        element = fea.ChElementShell4()
        element.SetNodes(node1, node2, node3, node4)
        element.SetMaterial(fea.ChMaterialShellKirchhoff(rho, E, nu, thickness))
        mesh.AddElement(element)


system.Add(mesh)





application = irr.ChIrrApp(
    system,
    "Tablecloth Folding Simulation",
    irr.dimension2du(1280, 720),
)
application.AddTypicalSky()
application.AddTypicalLights()
application.SetShowInfos(True)


vis_mesh = application.AssetBindMesh(mesh)
vis_mesh.SetWireframe(True)


application.AssetBindAll()
application.AssetUpdateAll()
application.SetCameraPosition(chrono.ChVectorD(width / 2, 0.5, length * 1.5), chrono.ChVectorD(width / 2, 0, 0))





time = 0
while time < total_time:
    
    system.DoStep(time_step)

    
    if system.GetChTime() > time:
        application.GetDevice().run()
        application.BeginScene()
        application.DrawAll()
        application.EndScene()
        time += 1 / output_frequency

print("Simulation complete.")