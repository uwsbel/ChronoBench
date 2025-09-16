import chrono as chrono
from chrono import IrrlichtVisualizer
import numpy as np


chrono.Initialize()


world = chrono.ChWorld()


solver = chrono.ChSolverPardisoMKL()


material = chrono.ChMaterialTool.IsotropicElastoplastic(1e9, 0.4, 1e-6)


L = 2.0
W = 1.0
H = 0.1


n_x = 10
n_y = 10
dx = L / (n_x - 1)
dy = W / (n_y - 1)


nodes = []
elements = []
for i in range(n_x):
    for j in range(n_y):
        x = i * dx
        y = j * dy
        node = chrono.ChNodeShell(x, y, 0, H)
        nodes.append(node)
        if i < n_x - 1 and j < n_y - 1:
            element = chrono.ChShellTriangle(node, nodes[(i + 1) * n_y + j], nodes[i * n_y + j + 1])
            elements.append(element)


for node in nodes:
    world.Add(node)
for element in elements:
    world.Add(element)


visualizer = IrrlichtVisualizer(world)
visualizer.SetAmbientLight(0.5, 0.5, 0.5)
visualizer.SetCameraPosition(0, 0, 10)
visualizer.SetCameraTarget(0, 0, 0)


world.SetGravity(0, 0, -9.81)
solver.SetMaxIterations(1000)
solver.SetTolerance(1e-6)


for i in range(100):
    world.DoStep()
    visualizer.Render()


chrono FINALIZE()