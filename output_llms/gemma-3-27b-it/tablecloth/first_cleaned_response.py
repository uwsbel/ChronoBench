import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


solver = chrono.ChSolverPardisoMKL()
system.SetSolver(solver)


tablecloth_length = 2.0
tablecloth_width = 2.0
num_x = 20  
num_y = 20  
thickness = 0.001


nodes = []
elements = []

for i in range(num_x):
    for j in range(num_y):
        x = -tablecloth_length / 2 + i * (tablecloth_length / (num_x - 1))
        y = -tablecloth_width / 2 + j * (tablecloth_width / (num_y - 1))
        nodes.append(chrono.ChVectorD(x, y, 0))

for i in range(num_x - 1):
    for j in range(num_y - 1):
        n1 = i * num_y + j
        n2 = (i + 1) * num_y + j
        n3 = (i + 1) * num_y + (j + 1)
        n4 = i * num_y + (j + 1)
        elements.append((n1, n2, n3, n4))


tablecloth = chrono.ChShell()
tablecloth.Set_Size(num_x, num_y)
tablecloth.Set_Thickness(thickness)


for i, node in enumerate(nodes):
    tablecloth.AddNode(node)

for element in elements:
    tablecloth.AddElement(*element)


youngs_modulus = 1000.0
poissons_ratio = 0.3
density = 1.0

material = chrono.ChMaterialSurfaceNSC()
material.SetYoungModulus(youngs_modulus)
material.SetPoissonRatio(poissons_ratio)
material.SetDensity(density)
tablecloth.SetMaterial(material)


system.Add(tablecloth)


tablecloth.SetFixed(0, True)  
tablecloth.SetFixed(num_y - 1, True)  
tablecloth.SetFixed((num_x - 1) * num_y, True)  
tablecloth.SetFixed((num_x - 1) * num_y + (num_y - 1), True)  



vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Tablecloth Folding')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 0, -2))
vis.AddTypicalLights()


time_step = 0.001
simulation_time = 5.0

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)

    if system.GetChTime() > simulation_time:
        break