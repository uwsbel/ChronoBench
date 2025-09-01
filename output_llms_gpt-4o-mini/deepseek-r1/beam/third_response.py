import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

print("Example: PyChrono using beam finite elements")

sys = chrono.ChSystemSMC()

# Create a mesh
mesh = fea.ChMesh()

# Create a beam section
msection = fea.ChBeamSectionEulerAdvanced()

beam_wy = 0.012
beam_wz = 0.025
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(0.01e9)
msection.SetShearModulus(0.01e9 * 0.3)
msection.SetRayleighDamping(0.000)
msection.SetCentroid(0, 0.02)
msection.SetShearCenter(0, 0.1)
msection.SetSectionRotation(45 * chrono.CH_DEG_TO_RAD)  # Corrected rotation

# Add Euler-Bernoulli beams
beam_L = 0.1

hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L * 2, 0, 0)))

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

# Apply forces and torques
hnode2.SetForce(chrono.ChVector3d(4, 2, 0))
hnode3.SetTorque(chrono.ChVector3d(0, -0.04, 0))

# Fix nodes using constraints
mtruss = chrono.ChBody()
mtruss.SetFixed(True)
sys.Add(mtruss)

# Correct constraints using ChLinkNodeFrame
# Fix hnode1: Y and Z translations, free X and rotations
link_hnode1 = chrono.ChLinkNodeFrame()
link_hnode1.Initialize(hnode1, mtruss)
link_hnode1.SetConstrainedCoords(False, True, True, False, False, False)
sys.Add(link_hnode1)

# Fix hnode3: All DOFs
link_hnode3 = chrono.ChLinkNodeFrame()
link_hnode3.Initialize(hnode3, mtruss)
link_hnode3.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(link_hnode3)

# Add beams using builder
builder = fea.ChBuilderBeamEuler()

# First beam
builder.BuildBeam(mesh, msection, 5,
                  chrono.ChVector3d(0, 0, -0.1),
                  chrono.ChVector3d(0.2, 0, -0.1),
                  chrono.ChVector3d(0, 1, 0))

# Fix A end (first node) and apply force to B end (last node)
builder.GetLastBeamNodes().front().SetFixed(True)  # Corrected front/back
builder.GetLastBeamNodes().back().SetForce(chrono.ChVector3d(0, -1, 0))

# Add second beam segment as per instructions
builder.BuildBeam(mesh, msection, 5,
                  chrono.ChVector3d(0.2, 0, -0.1),  # Start from previous beam's endpoint
                  chrono.ChVector3d(0.2, 0.1, -0.1),  # New endpoint
                  chrono.ChVector3d(0, 1, 0))

# Disable gravity for FEA
mesh.SetAutomaticGravity(False)

sys.Add(mesh)

# Visualization
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

# Irrlicht setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.1, 0.1, 0.2))
vis.AddTypicalLights()

# Solver setup
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.001)