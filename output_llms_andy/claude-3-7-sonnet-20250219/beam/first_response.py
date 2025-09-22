import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import numpy as np

# Global settings
beam_density = 7800      # Steel density [kg/m^3]
beam_radius = 0.01       # Beam radius [m]
beam_Young = 2.1e11      # Young's modulus [Pa]
beam_Poisson = 0.3       # Poisson ratio
beam_length = 1.0        # Length of the beam structure [m]
n_elements = 8           # Number of beam elements along the length

# Initialize the physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# Create a mesh for beam FEA
mesh = fea.ChMesh()

# Create material for the beams
beam_material = fea.ChBeamSectionAdvanced()
beam_material.SetDensity(beam_density)
beam_material.SetYoungModulus(beam_Young)
beam_material.SetGshearModulus(beam_Young / (2 * (1 + beam_Poisson)))
beam_material.SetBeamRaleyghDamping(0.01)
beam_material.SetAsCircularSection(beam_radius)

# Create nodes and beams
nodes = []
for i in range(n_elements + 1):
    # Position along the beam length
    x = i * (beam_length / n_elements)
    
    # Create node
    node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(
        chrono.ChVectorD(x, 0, 0),
        chrono.ChQuaternionD(1, 0, 0, 0)
    ))
    
    # Set node mass and inertia
    node.SetMass(0)  # Mass will be automatically computed from beam properties
    nodes.append(node)
    mesh.AddNode(node)
    
    # First node is fixed to the ground
    if i == 0:
        node_constraint = fea.ChLinkMateGeneric()
        node_constraint.Initialize(node, system.GetBodyList()[0], 
                               chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
        node_constraint.SetConstrainedCoords(True, True, True, True, True, True)
        system.Add(node_constraint)

# Create beam elements connecting the nodes
for i in range(n_elements):
    beam_element = fea.ChElementBeamEuler()
    beam_element.SetNodes(nodes[i], nodes[i + 1])
    beam_element.SetSection(beam_material)
    mesh.AddElement(beam_element)

# Add mesh to the system
system.Add(mesh)

# Set up integrator for the mesh
mesh.SetAutomaticGravity(True)
system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)

# Apply a force at the end node to demonstrate bending
end_node = nodes[-1]
end_node.SetForce(chrono.ChVectorD(0, -50, 0))  # Vertical force of 50N

# Create visualization
vis_mat = chrono.ChVisualMaterial()
vis_mat.SetDiffuseColor(chrono.ChColor(0.4, 0.4, 0.4))

# Set visualization assets for beams
for i in range(n_elements):
    beam_element = mesh.GetElement(i)
    beam_element.AddVisualShape(chrono.ChVisualShapeSegment())

# Initialize Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Beam Finite Element Demo")
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(beam_length/2, 0.5, -1), chrono.ChVectorD(beam_length/2, 0, 0))
vis.AddTypicalLights()

# Main simulation loop
step_size = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(step_size)