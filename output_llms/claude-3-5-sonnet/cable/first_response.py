import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# Initialize the ChSystem
system = chrono.ChSystemNSC()

# Set gravity
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the ground
ground = chrono.ChBodyEasyBox(1, 0.1, 1, 1000, True)
ground.SetBodyFixed(True)
system.Add(ground)

# Parameters for the beam
beam_length = 1.0
beam_diameter = 0.01
num_elements = 10
density = 1000

# Create the beam using ANCF cable elements
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

# Fix the first node (hinge)
beam_nodes[0].SetFixed(True)

# Create the visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("ANCF Beam Simulation")
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0.5, 0.5, 2))
vis.AddTypicalLights()

# Simulation loop
time_step = 0.001
while vis.Run():
    system.DoStepDynamics(time_step)
    
    # Visualize beam nodes
    for node in beam_nodes:
        vis.GetDevice().drawSphere(node.GetPos(), 0.01, chronoirr.SColor(255, 0, 0, 255))
    
    # Visualize beam elements
    for element in beam_elements:
        start = element.GetNodeA().GetPos()
        end = element.GetNodeB().GetPos()
        vis.GetDevice().drawLine(start, end, chronoirr.SColor(255, 255, 0, 0))
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

vis.GetDevice().closeDevice()