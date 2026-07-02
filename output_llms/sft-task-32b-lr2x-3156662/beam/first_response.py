import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr
import numpy as np

# Initialize the physical system and solver
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  # No gravity

# Create the mesh and add it to the system
mesh = fea.ChMesh()
system.Add(mesh)

# Define beam section properties
section = fea.ChBeamSectionCable()
section.SetYoungModulus(200e9)  # Steel Young's modulus
section.SetPoissonRatio(0.3)
section.SetDiameter(0.05)  # 5 cm diameter
section.SetRho(7800)  # Density of steel

# Create nodes and beam elements using ChBuilderBeamIGA
builder = fea.ChBuilderBeamIGA()
num_elements = 10
length = 1.0
builder.BuildBeam(mesh, num_elements, length, section, 
                  chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(length, 0, 0), chrono.CH_MATTELLA_X)

# Fix the first node
fixed_node = mesh.GetNode(0)
fixed_frame = chrono.ChFrameMoving()
fixed_link = chrono.ChLinkPointFrame()
fixed_link.Initialize(fixed_node, fixed_frame)
system.Add(fixed_link)

# Apply a downward force to the last node
last_node = mesh.GetNode(mesh.GetNumNodes() - 1)
last_node.SetForce(chrono.ChVector3d(0, -1000, 0))  # 1000 N downward force

# Add visualization for beam elements
for element in mesh.GetElements():
    vis = chrono.ChVisualShapeFEAbeam()
    vis.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
    vis.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
    vis.SetSymbolsThickness(0.03)
    vis.SetSymbolsScale(0.01)
    vis.SetZbufferHide(False)
    element.AddVisualShapeFEA(vis)

# Set up Irrlicht visualization
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("PyChrono Beam FEA Simulation")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2, -4, 2), chrono.ChVector3d(length / 2, 0, 0))
vis.AddTypicalLights()

# Simulation loop parameters
time_step = 0.001
simulation_time = 5.0
step_count = int(simulation_time / time_step)

# Run the simulation
while vis.Run() and step_count > 0:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)
    step_count -= 1