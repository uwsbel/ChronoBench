import pychrono as chrono
import pychrono.physics as physics
import pychrono.visualization as visualization
import pychrono.solver as solver
import numpy as np


chrono.Init()


world = chrono.World()


material = chrono.Material(chrono.IsotropicKirchhoff())


shell = chrono.ShellElement(material)


mesh = chrono.Mesh(shell, 100, 100)





visualizer = visualization.IrrlichtVisualizer()
visualizer.SetWorld(world)


solver = solver.PardisoMKL()


world.Add(shell)


world.Run()