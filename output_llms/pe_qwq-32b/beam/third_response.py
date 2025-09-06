import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

print("Example: PyChrono using beam finite elements")

sys = chrono.ChSystemSMC()

# Create a mesh container
mesh = fea.ChMesh()

# Create beam section properties
msection = fea.ChBeamSectionEulerAdvanced()
beam_wy = 0.012
beam_wz = 0.025
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(0.01e9)
msection.SetShearModulus(0.01e9 * 0.3)
msection.SetRayleighDamping(0.000)
msection.SetSectionRotation(45 * chrono.CH_C_DEG_TO_RAD)  # Fixed conversion

# Create initial nodes and elements
beam_L = 0.1
hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(beam_L, 0, 0)))
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(beam_L * 2, 0, 0)))

mesh.AddNode(hnode1)
mesh.AddNode(hnode2)
mesh.AddNode(hnode3)

belement1 = fea.ChElementBeamEuler()
belement1.SetNodes(hnode1, hnode2)
belement1.SetSection(msection)
mesh.AddElement(belement1)

belement2 = fea.ChElementBeamEuler()
belement2.SetNodes(hnode2, hnode3)
belement2.SetSection(msection)
mesh.AddElement(belement2)

# Apply forces/torques to nodes
hnode2.SetForce(chrono.ChVectorD(4, 2, 0))
hnode3.SetTorque(chrono.ChVectorD(0, -0.04, 0))

# Fix initial nodes
hnode1.SetFixed(True)
hnode3.SetFixed(True)

# Use builder for Euler beams
builder = fea.ChBuilderBeamEuler()

# First beam using builder
builder.BuildBeam(
    mesh,
    msection,
    5,
    chrono.ChVectorD(0, 0, -0.1),
    chrono.ChVectorD(0.2, 0, -0.1),
    chrono.ChVectorD(0, 1, 0)
)

# Fix last node of first builder beam
builder.GetLastBeamNodes().back().SetFixed(True)
builder.GetLastBeamNodes().front().SetForce(chrono.ChVectorD(0, -1, 0))

# Add new beam segment per instructions
builder.BuildBeam(
    mesh,
    msection,
    5,
    chrono.ChVectorD(0.2, 0, -0.1),  # Start at end of previous beam
    chrono.ChVectorD(0.2, 0.1, -0.1),  # New endpoint
    chrono.ChVectorD(0, 1, 0)
)

# Disable gravity for FEA elements
mesh.SetAutomaticGravity(False)
sys.Add(mesh)

# Visualization setup
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
visualizebeamA.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(visualizebeamA)

visualizebeamC = chrono.ChVisualShapeFEA(mesh)
visualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
visualizebeamC.SetSymbolsThickness(0.006)
visualizebeamC.SetSymbolsScale(0.01)
mesh.AddVisualShapeFEA(visualizebeamC)

# Setup Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.1, 0.1, 0.2))
vis.AddTypicalLights()

# Use MKL solver for better performance
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.001)