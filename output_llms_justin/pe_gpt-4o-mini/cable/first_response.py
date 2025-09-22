import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea

# Initialize the PyChrono system using SMC for smoother transitions
sys = chrono.ChSystemSMC()

# Set the gravity in the simulation
sys.Set_G_acc(chrono.ChVector(0, -9.81, 0))

# Create a mesh for the ANCF cable beam
mesh = fea.ChMesh()

# Define parameters for the beam
length = 5.0  # Length of the beam
num_elements = 10  # Number of ANCF cable elements
element_length = length / num_elements  # Length of each element
diameter = 0.1  # Diameter of the beam

# Create the ANCF cable elements and nodes
for i in range(num_elements + 1):
    # Create a node at each position
    node = fea.ChNodeFEAxyzrot(chrono.ChFrame(chrono.ChVector3d(i * element_length, 0, 0), chrono.QUNIT))
    mesh.AddNode(node)

# Create elements between the nodes
for i in range(num_elements):
    element = fea.ChElementCableANCF()
    element.SetNodes(mesh.GetNode(i), mesh.GetNode(i + 1))
    element.SetDiameter(diameter)
    element.SetDensity(1000)  # Density of the cable
    mesh.AddElement(element)

# Add the mesh to the system
sys.Add(mesh)

# Fix the first node to the ground (hinge)
first_node = mesh.GetNode(0)
first_node.SetFixed(True)

# Setup the visualization using the Irrlicht engine
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ANCF Cable Beam Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(0, 3, 10))
vis.AddTypicalLights()

# Simulation loop
time_step = 0.01
while vis.Run():
    sys.DoStepDynamics(time_step)

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Optionally, visualize the nodal positions
    for i in range(num_elements + 1):
        node_pos = mesh.GetNode(i).GetPos()
        print(f'Node {i} Position: {node_pos}')