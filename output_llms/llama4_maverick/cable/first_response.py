import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# Initialize the PyChrono environment
chrono.SetChronoDataPath("./chrono_data/")

# Create a Chrono system
sys = chrono.ChSystemSMC()

# Create a material for the cable elements
cable_mat = chrono.ChMaterialSurfaceSMC()
cable_mat.SetYoungModulus(2e6)  # Young's modulus (Pa)
cable_mat.SetPoissonRatio(0.3)   # Poisson's ratio
cable_mat.SetFriction(0.5)      # Friction coefficient
cable_mat.SetRestitution(0.2)   # Restitution coefficient

# Define the number of elements and nodes in the beam
num_elements = 10
num_nodes = num_elements + 1

# Create the nodes
nodes = []
for i in range(num_nodes):
    x = i * 1.0 / num_elements  # Node position along the x-axis
    node = chrono.ChNodeFEAxyzD(chrono.ChVectorD(x, 0, 0), chrono.ChVectorD(1, 0, 0))
    node.SetMass(0.1)  # Node mass (kg)
    sys.AddNode(node)
    nodes.append(node)

# Create the ANCF cable elements
elements = []
for i in range(num_elements):
    element = chrono.ChElementCableANCF()
    element.SetNodes(nodes[i], nodes[i+1])
    element.SetMaterial(cable_mat)
    element.SetSection(chrono.ChBeamSectionCable(0.01))  # Cable section radius (m)
    sys.AddElement(element)
    elements.append(element)

# Create a fixed body to represent the ground
ground = chrono.ChBody()
ground.SetBodyFixed(True)  # Fix the ground body
sys.AddBody(ground)

# Create a revolute joint between the first node and the ground
revolute_joint = chrono.ChLinkRevolute()
revolute_joint.Initialize(nodes[0], ground, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
sys.AddLink(revolute_joint)

# Set gravity
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity (m/s^2)

# Create an Irrlicht application
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ANCF Cable Beam Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1, 1, 2))
vis.AddTypicalLights()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    chronoirr.ChIrrTools.drawGrid(vis, 1, 1, 10, 10, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), chronoirr.SColor(255, 255, 255), True)
    chronoirr.ChIrrTools.drawColumn(vis, 1, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0), 10, chrono.ChCoordsysD(), chronoirr.SColor(255, 0, 0))
    
    # Draw the beam's deformation and nodal positions
    for i in range(num_elements):
        node1 = nodes[i].GetPos()
        node2 = nodes[i+1].GetPos()
        vis.GetVideoDriver().draw3DLine(chronoirr.ChVector_to_SVector(node1), chronoirr.ChVector_to_SVector(node2), chronoirr.SColor(255, 0, 255))
    for node in nodes:
        vis.GetVideoDriver().draw3DLine(chronoirr.ChVector_to_SVector(node.GetPos() - chrono.ChVectorD(0.01, 0, 0)), chronoirr.ChVector_to_SVector(node.GetPos() + chrono.ChVectorD(0.01, 0, 0)), chronoirr.SColor(255, 255, 0))
        vis.GetVideoDriver().draw3DLine(chronoirr.ChVector_to_SVector(node.GetPos() - chrono.ChVectorD(0, 0.01, 0)), chronoirr.ChVector_to_SVector(node.GetPos() + chrono.ChVectorD(0, 0.01, 0)), chronoirr.SColor(255, 255, 0))
        vis.GetVideoDriver().draw3DLine(chronoirr.ChVector_to_SVector(node.GetPos() - chrono.ChVectorD(0, 0, 0.01)), chronoirr.ChVector_to_SVector(node.GetPos() + chrono.ChVectorD(0, 0, 0.01)), chronoirr.SColor(255, 255, 0))
    
    vis.EndScene()
    sys.DoStepDynamics(0.001)  # Time step (s)