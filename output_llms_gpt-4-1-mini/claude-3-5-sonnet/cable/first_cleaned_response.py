import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(1, 0.1, 1, 1000, True)
ground.SetBodyFixed(True)
system.Add(ground)


beam_length = 1.0
beam_diameter = 0.01
num_elements = 10
density = 1000


beam_nodes = []
beam_elements = []

for i in range(num_elements + 1):
    node = chrono.ChNodeFEAxyzD(chrono.ChVectorD(i * beam_length / num_elements, 0, 0))
    beam_nodes.append(node)
    system.Add(node)

    if i > 0:
        element = chrono.ChElementCableANCF()
        element.SetNodes(beam_nodes[i-1], beam_nodes[i])
        element.SetDensity(density)
        element.SetSection(np.pi * beam_diameter**2 / 4)
        element.SetYoungModulus(1e7)
        beam_elements.append(element)
        system.Add(element)


beam_nodes[0].SetFixed(True)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("ANCF Beam Simulation")
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0.5, 0.5, 2))
vis.AddTypicalLights()


time_step = 0.001
while vis.Run():
    system.DoStepDynamics(time_step)
    
    
    for node in beam_nodes:
        vis.GetDevice().drawSphere(node.GetPos(), 0.01, chronoirr.SColor(255, 0, 0, 255))
    
    
    for element in beam_elements:
        start = element.GetNodeA().GetPos()
        end = element.GetNodeB().GetPos()
        vis.GetDevice().drawLine(start, end, chronoirr.SColor(255, 255, 0, 0))
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

vis.GetDevice().closeDevice()