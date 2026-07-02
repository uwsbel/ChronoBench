import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

print("Example: PyChrono using beam finite elements")

# Create the physical system
sys = chrono.ChSystemSMC()

# Create a mesh for FEA elements
mesh = fea.ChMesh()

# Define beam section properties (Euler-Bernoulli)
msection = fea.ChBeamSectionEulerAdvanced()
beam_wy = 0.012
beam_wz = 0.025
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(0.01e9)
msection.SetShearModulus(0.01e9 * 0.3)
msection.SetRayleighDamping(0.000)
msection.SetCentroid(0, 0.02)
msection.SetShearCenter(0, 0.1)
msection.SetSectionRotation(45 * chrono.CH_RAD_TO_DEG)

# Create beam using ChBuilderBeamEuler helper
builder = fea.ChBuilderBeamEuler()
builder.SetSection(msection)
builder.SetYDir(chrono.ChVector<>(0, 1, 0))  # Y-up orientation

# Build beam from (0, 0, -0.1) to (0.2, 0, -0.1) with 5 elements
beam = builder.BuildBeam(chrono.ChVector<>(0, 0, -0.1), chrono.ChVector<>(0.2, 0, -0.1), 5)

# Add beam to mesh
mesh.AddElement(beam)

# Apply force to first node of the beam
first_node = beam.GetNode1()
first_node.SetForce(chrono.ChVector3d(0, -1, 0))

# Fix last node using SetFixed (as per instruction 3)
last_node = beam.GetNode2()
last_node.SetFixed(True)

# Add mesh to the system
sys.Add(mesh)

# Visualization settings
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)

visualizebeamC = chrono.ChVisualShapeFEA(mesh)
visualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
visualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
visualizebeamC.SetSymbolsThickness(0.006)
visualizebeamC.SetSymbolsScale(0.01)
visualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualizebeamC)

# Create visualization window
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.1, 0.1, 0.2))
vis.AddTypicalLights()

# Set MKL Pardiso solver
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.001)