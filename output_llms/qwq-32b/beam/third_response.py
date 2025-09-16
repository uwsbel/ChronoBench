import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

print("Example: PyChrono using beam finite elements")

sys = chrono.ChSystemSMC()

## Create a mesh, that is a container for groups of elements and their referenced nodes.
mesh = fea.ChMesh()

## Create a section, i.e. thickness and material properties for beams. This will be shared among some beams.
msection = fea.ChBeamSectionEulerAdvanced()

beam_wy = 0.012
beam_wz = 0.025
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(0.01e9)
msection.SetShearModulus(0.01e9 * 0.3)
msection.SetRayleighDamping(0.000)
msection.SetCentroid(0, 0.02)
msection.SetShearCenter(0, 0.1)
msection.SetSectionRotation(45 * chrono.CH_DEG_TO_RAD)  # Fixed rotation conversion

beam_L = 0.1

# Create nodes with corrected frame class name
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

# Apply forces/torques
hnode2.SetForce(chrono.ChVectorD(4, 2, 0))
hnode3.SetTorque(chrono.ChVectorD(0, -0.04, 0))

# Fix nodes using SetFixed instead of invalid constraints
hnode1.SetFixed(True)
hnode3.SetFixed(True)

# Remove invalid constraints (ChLinkMateGeneric with nodes)
# ... (original constraint code is removed)

# Add beams via builder
builder = fea.ChBuilderBeamEuler()

# Existing beam
builder.BuildBeam(
    mesh,
    msection,
    5,
    chrono.ChVectorD(0, 0, -0.1),
    chrono.ChVectorD(0.2, 0, -0.1),
    chrono.ChVectorD(0, 1, 0)
)

# New beam added per instructions
builder.BuildBeam(
    mesh,
    msection,
    5,
    chrono.ChVectorD(0.2, 0, -0.1),
    chrono.ChVectorD(0.2, 0.1, -0.1),
    chrono.ChVectorD(0, 1, 0)
)

# Fix and force application for new beam's endpoints
# (Optional: if needed, but not specified in instructions)
# builder.GetLastBeamNodes().back().SetFixed(True)
# builder.GetLastBeamNodes().front().SetForce(...)

mesh.SetAutomaticGravity(False)
sys.Add(mesh)

# Add visual assets (unchanged)
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

# Setup visualization and solver
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.1, 0.1, 0.2))
vis.AddTypicalLights()

sys.SetSolver(mkl.ChSolverPardisomkl())

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.001)