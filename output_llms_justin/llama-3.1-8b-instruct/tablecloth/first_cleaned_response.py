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
        z = H / 2
        node = chrono.ChBodyEasyBox(L/10, W/10, H/10, material, True, False)
        node.SetPos(chrono.ChVector(x, y, z))
        world.Add(node)
        nodes.append(node)
for i in range(n_x - 1):
    for j in range(n_y - 1):
        node1 = nodes[i * n_y + j]
        node2 = nodes[i * n_y + j + 1]
        node3 = nodes[(i + 1) * n_y + j + 1]
        node4 = nodes[(i + 1) * n_y + j]
        element = chrono.ChShellEasyBox(L/10, W/10, H/10, node1, node2, node3, node4)
        world.Add(element)
        elements.append(element)


visualizer = IrrlichtVisualizer()
visualizer.SetWorld(world)
visualizer.SetCameraPosition(chrono.ChVector(0, 5, 5))
visualizer.SetCameraTarget(chrono.ChVector(0, 0, 0))
visualizer.Render()


t = 0
dt = 0.01
while t < 10:
    world.DoStep(dt)
    visualizer.Render()
    t += dt


chrono FINALIZE()