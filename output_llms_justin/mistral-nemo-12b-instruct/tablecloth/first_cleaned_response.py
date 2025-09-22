import pychrono as chrono
import numpy as np


chrono.setChronoDataPath("path/to/chrono_data")  
chrono.setChronoLogLevel(chrono.LOG_LEVEL_ERROR)


simulation = chrono.ChSimulation()
system = simulation.getSystem()
solver = chrono.ChSolverPardisoMKL()
system.SetSolver(solver)


material = chrono.ChMaterialSurfaceNSC()
material.SetYoungModulus(1e6)  
material.SetPoissonRatio(0.3)  
material.SetThickness(0.01)  


num_nodes_u = 10
num_nodes_v = 10
nodes = []
elements = []
for u in range(num_nodes_u):
    for v in range(num_nodes_v):
        node = chrono.ChNodeShell()
        node.SetPos(chrono.ChVectorD(u * 0.1, v * 0.1, 0))  
        nodes.append(node)
        system.Add(node)

        
        element = chrono.ChElementShell()
        element.SetNodes(nodes[2 * u + v], nodes[2 * u + v + 1])
        element.SetMaterial(material)
        elements.append(element)
        system.Add(element)


vis = chrono.ChVisualSystemIrrlicht()
vis.Attach(system)
vis.Initialize()
vis.SetWindowTitle("Tablecloth Folding Simulation")


for node in nodes:
    node.SetForce(chrono.ChVectorD(0, 0, -1))  


while vis.Run():
    simulation.DoStepDynamics(0.01)  
    vis.Render()
    vis.WaitForFrameEnd()