# Import required modules
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# Print introductory statement
print("Example: PyChrono using Euler-Bernoulli beam finite elements")

# Create the physical system
sys = chrono.ChSystemSMC()

# Create a mesh for the finite element analysis
mesh = fea.ChMesh()

# Create a Euler-Bernoulli beam section
msection = fea.ChBeamSectionEuler()
# Set the cross-sectional properties of the beam
msection.SetAsRectangularSection(0.01, 0.03)  # beam_wy and beam_wz

# Create a beam builder helper object
builder = fea.ChBuilderBeamEuler()

# Define beam properties
beam_length = 0.2
num_elements = 5
builder.SetMaterialProperties(msection)  # Set material properties from section
builder.SetLength(beam_length)  # Set total length of the beam
builder.SetDirection(chrono.ChVector3d(0, 1, 0))  # Set Y-up direction
builder.SetFirstCoordinate(chrono.ChVector3d(0, 0, -0.1))  # Set starting coordinates (X, Y, Z)
builder.SetLastCoordinate(chrono.ChVector3d(beam_length, 0, -0.1))  # Set ending coordinates (X, Y, Z)

# Create the beam
beam = builder.BuildBeam()

# Add beam to mesh
mesh.AddElement(beam)

# Add nodes to mesh (nodes are automatically added by the beam builder)
# mesh.AddNode(beam.GetNodes()[0])
# mesh.AddNode(beam.GetNodes()[1])

# Apply forces to nodes (applying forces directly is not recommended, use constraints)
# Force at first node (node A)
force_A = chrono.ChVector3d(0, -1, 0)
beam.GetNodes()[0].SetForce(force_A)

# Fix the last node (node B) using constraints instead of direct fixing
# Create a fixed constraint for node B
constraint_B = chrono.ChLinkMateGeneric()
constraint_B.Initialize(beam.GetNodes()[1], sys, False, beam.GetNodes()[1].Frame(), beam.GetNodes()[1].Frame())
constraint_B.SetConstrainedCoords(True, True, True)  # Constrain x, y, z translations

# Add constraint to system
sys.Add(constraint_B)

# Create visualization for the beam
visualize_beam = chrono.ChVisualShapeFEA(mesh)
visualize_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # Visualize bending moments
visualize_beam.SetColorscaleMinMax(-0.4, 0.4)  # Color scale limits
visualize_beam.SetSmoothFaces(True)  # Smooth faces
visualize_beam.SetWireframe(False)  # Disable wireframe mode
mesh.AddVisualShapeFEA(visualize_beam)

# Create visualization for nodes
visualize_nodes = chrono.ChVisualShapeFEA(mesh)
visualize_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)  # Visualize coordinate systems
visualize_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  # No data type visualization
visualize_nodes.SetSymbolsThickness(0.006)  # Symbol thickness
visualize_nodes.SetSymbolsScale(0.01)  # Symbol scale
visualize_nodes.SetZbufferHide(False)  # Do not hide symbols
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