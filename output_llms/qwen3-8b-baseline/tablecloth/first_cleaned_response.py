import pychrono as chrono
from pychrono import irrlicht
from pychrono import ChMaterialShellKirchhoff


system = chrono.ChSystemNSC()
system.SetSolverType(chrono.ChSolverPardisoMKL())
system.SetGravity(chrono.ChVector(0, 0, -9.81))


material = ChMaterialShellKirchhoff()
material.SetE(1e6)  
material.Setnu(0.3)  
material.SetThickness(0.002)  


nx, ny = 10, 10
dx, dy = 0.1, 0.1  

nodes = []
for i in range(nx):
    for j in range(ny):
        x = i * dx
        y = j * dy
        z = 0
        node = chrono.ChNodeShellVertex(chrono.ChVector(x, y, z))
        nodes.append(node)
        system.Add(node)

elements = []
for i in range(nx - 1):
    for j in range(ny - 1):
        idx1 = i * ny + j
        idx2 = (i + 1) * ny + j
        idx3 = (i + 1) * ny + j + 1
        idx4 = i * ny + j + 1
        element = chrono.ChShellElement()
        element.SetNodes(nodes[idx1], nodes[idx2], nodes[idx3], nodes[idx4])
        element.SetMaterial(material)
        system.Add(element)
        elements.append(element)


vis = irrlicht.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Tablecloth Folding Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVector(0, 1, 2), chrono.ChVector(0, 0, 0))
vis.SetCameraLookAtPoint(chrono.ChVector(0, 0, 0))


time_step = 1e-3
while vis.Run():
    vis.BeginScene()
    vis.Render()
    system.DoStepDynamics(time_step)
    vis.EndScene()