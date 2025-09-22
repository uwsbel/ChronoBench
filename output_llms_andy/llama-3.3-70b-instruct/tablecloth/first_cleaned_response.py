import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.mesh as chronomesh
import numpy as np


sys = chrono.ChSystem()


sys.SetDefaultSolver(chrono.ChSolverPardisoMKL())


ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
ground.SetBodyFixed(True)
sys.Add(ground)


E = 1e6  
nu = 0.3  
thickness = 0.1  
density = 1000  


nx, ny = 20, 20  
Lx, Ly = 5, 5  
nodes = []
elements = []


for i in range(nx + 1):
    for j in range(ny + 1):
        x = i * Lx / nx
        y = j * Ly / ny
        node = chrono.ChNodeFEAxyz(chrono.ChVectorD(x, y, 0))
        node.SetMass(density * thickness * (Lx / nx) * (Ly / ny))
        sys.Add(node)
        nodes.append(node)


for i in range(nx):
    for j in range(ny):
        n1 = nodes[i * (ny + 1) + j]
        n2 = nodes[i * (ny + 1) + j + 1]
        n3 = nodes[(i + 1) * (ny + 1) + j + 1]
        n4 = nodes[(i + 1) * (ny + 1) + j]
        element = chrono.ChElementShellSK(chrono.ChVectorD(0, 0, 0), n1, n2, n3, n4, thickness, E, nu)
        sys.Add(element)
        elements.append(element)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 0, 10))
vis.AddTypicalLights()


time_step = 1e-3
end_time = 10

while sys.GetChTime() < end_time:
    sys.DoStepDynamics(time_step)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()

    
    if sys.GetChTime() < 1:
        force = chrono.ChForce()
        force.SetF(chrono.ChVectorD(0, -100, 0))
        nodes[nx * (ny + 1) // 2].AddForce(force)


sys.Delete()