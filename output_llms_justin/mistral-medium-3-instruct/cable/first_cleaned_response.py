import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


beam_length = 1.0  
num_elements = 10  
num_nodes = num_elements + 1  


nodes = []
for i in range(num_nodes):
    
    x = i * beam_length / num_elements
    node = chrono.ChNodeFEAxyzD(x, 0, 0)
    system.Add(node)
    nodes.append(node)


elements = []
for i in range(num_elements):
    element = chrono.ChElementCableANCF()
    element.SetNodes(nodes[i], nodes[i+1])

    
    element.SetDiameter(0.02)  
    element.SetYoungModulus(2.1e11)  
    element.SetDensity(7850)  
    element.SetRayleighDamping(0.01)  

    system.Add(element)
    elements.append(element)


constraint = chrono.ChLinkLockLock()
constraint.Initialize(nodes[0], chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
system.Add(constraint)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("ANCF Cable Beam Simulation")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.5, 0.5, 2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, 5, 2), chrono.ChVectorD(0, 0, 0), 3, 1, 10, 40, 512)


for element in elements:
    vis.AddFEAelement(element)


time_step = 0.001
vis.SetSimulationStep(time_step)

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)

    
    if system.GetChTime() > 0.1 and system.GetChTime() < 0.11:
        print("Time:", system.GetChTime())
        print("Node 0 position:", nodes[0].GetPos())
        print("Node 5 position:", nodes[5].GetPos())