# Import required modules
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# Print introductory statement
print("Example: PyChrono using Euler-Bernoulli beam finite elements")

# Create the physical system
sys = chrono.ChSystemSMC()

# Create a mesh container
mesh = fea.ChMesh()

# Create a fixed Euler-Bernoulli beam section
# Using ChBuilderBeamEuler helper object
beam_builder = fea.ChBuilderBeamEuler()
beam_builder.SetLength(0.2)  # Total length of the beam
beam_builder.SetDirection(chrono.ChVector3d(0, 0, -1))  # Direction of the beam (negative z-axis)
beam_builder.SetYoungModulus(0.01e9)  # Young's modulus
beam_builder.SetNumberOfElements(5)  # Number of elements
beam_builder.SetFirstYDirection(chrono.ChVector3d(0, 1, 0))  # 'Y' direction for beam
beam_builder.SetSectionRotation(0)  # No rotation

# Create the beam from node A to node B
beam = beam_builder.BuildBeam()

# Add nodes to the mesh
# Node A at (0, 0, -0.1)
node_A = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, -0.1)))
# Node B at (0.2, 0, -0.1)
node_B = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0.2, 0, -0.1)))

# Add nodes to the mesh
mesh.AddNode(node_A)
mesh.AddNode(node_B)

# Create beam elements and add to mesh
beam_element = fea.ChElementBeamEuler()
beam_element.SetNodes(node_A, node_B)
beam_element.SetSection(beam_builder.GetSection())
mesh.AddElement(beam_element)

# Fix the last node (node B) using constraints
constraint_B = chrono.ChLinkMateGeneric()
constraint_B.Initialize(node_B, sys.GetFixedBody(), False, node_B.Frame(), node_B.Frame())
constraint_B.SetConstrainedCoords(True, True, True)
sys.Add(constraint_B)

# Apply a force to the first node (node A)
node_A.SetForce(chrono.ChVector3d(0, -1, 0))

# Create visualization for the beam
visualize_beam = chrono.ChVisualShapeFEA(mesh)
visualize_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualize_beam.SetColorscaleMinMax(-0.4, 0.4)
visualize_beam.SetSmoothFaces(True)
visualize_beam.SetWireframe(False)
mesh.AddVisualShapeFEA(visualize_beam)

# Create visualization for nodes
visualize_nodes = chrono.ChVisualShapeFEA(mesh)
visualize_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
visualize_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
visualize_nodes.SetSymbolsThickness(0.006)
visualize_nodes.SetSymbolsScale(0.01)
visualize_nodes.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualize_nodes)

# Create visualization window
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA Euler-Bernoulli Beam')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.1, 0.1, 0.2))
vis.AddTypicalLights()

# Set solver to MKL Pardiso
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.001)