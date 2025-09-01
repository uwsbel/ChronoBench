import math
import numpy as np
import pychrono as chrono
import pychrono.fea as fea
import pychrono irrlicht as chronoirr

# Initialize the Chrono system with SMC solver for FEA
sys = chrono.ChSystemSMC()
sys.SetSolverType(chrono.ChSolver.Type_BARZILAI_BORWEIN)

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam FEA Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))
vis.AddTypicalLights()

# Beam properties
beam_length = 5.0
beam_width = 0.1
beam_height = 0.05
young_modulus = 2.0e11
shear_modulus = young_modulus / 2.6
density = 8000.0

# Create the beam mesh
mesh = fea.ChMesh()

# Create beam section properties
section = fea.ChBeamSectionEulerAdvanced()
section.SetAsRectangularSection(beam_width, beam_height)
section.SetYoungModulus(young_modulus)
section.SetShearModulus(shear_modulus)
section.SetRayleighDamping(0.001)

# Create nodes
node1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
node2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_length, 0, 0)))

# Add nodes to mesh
mesh.AddNode(node1)
mesh.AddNode(node2)

# Create beam element
element = fea.ChElementBeamEuler()
element.SetNodes(node1, node2)
element.SetSection(section)
mesh.AddElement(element)

# Add visualization to nodes
node1_shape = chrono.ChVisualShapeSphere(0.1)
node1_shape.SetColor(chrono.ChColor(0, 1, 0))
node1.AddVisualShape(node1_shape)

node2_shape = chrono.ChVisualShapeSphere(0.1)
node2_shape.SetColor(chrono.ChColor(0, 1, 0))
node2.AddVisualShape(node2_shape)

# Add visualization to beam
beam_shape = chrono.ChVisualShapeBox(beam_length, beam_width, beam_height)
beam_shape.SetColor(chrono.ChColor(0, 0, 1))
element.AddVisualShape(beam_shape)

# Add ground body for reference
ground = chrono.ChBody()
ground.SetPos(chrono.ChVector3d(0, -1, 0))
ground.SetFixed(True)
ground_shape = chrono.ChVisualShapeBox(1, 1, 1)
ground_shape.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
ground.AddVisualShape(ground_shape)
sys.Add(ground)

# Add the mesh to the system
sys.Add(mesh)

# Simulation parameters
simulation_time = 10.0
time_step = 0.001

# Simulation loop
current_time = 0.0
while vis.Run() and current_time < simulation_time:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    sys.DoStepDynamics(time_step)
    current_time += time_step

# Cleanup
vis.Close()