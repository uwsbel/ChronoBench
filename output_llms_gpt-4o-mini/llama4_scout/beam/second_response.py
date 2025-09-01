import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

print("Example: PyChrono using beam finite elements")

# Create the physical system that will be simulated.
sys = chrono.ChSystemSMC()

# Create a mesh, which is a container for elements and their referenced nodes.
mesh = fea.ChMesh()

# Create a section object for beam properties. This will define the characteristics of all beams that use this section.
msection = fea.ChBeamSectionEulerAdvanced()

# Set the width and height of the rectangular section of the beam.
beam_wy = 0.012
beam_wz = 0.025
msection.SetAsRectangularSection(beam_wy, beam_wz)

# Set the material properties of the beam.
msection.SetYoungModulus(0.01e9)
msection.SetShearModulus(0.01e9 * 0.3)
msection.SetRayleighDamping(0.000)
msection.SetCentroid(0, 0.02)
msection.SetShearCenter(0, 0.1)
msection.SetSectionRotation(45 * chrono.CH_RAD_TO_DEG)

# Add a Section on Euler-Bernoulli Beams
# =====================================================
## Euler-Bernoulli beam setup
builder = fea.ChBuilderBeamEuler()
builder.SetStartPoint(chrono.ChVector3d(0, 0, -0.1))
builder.SetEndPoint(chrono.ChVector3d(0.2, 0, -0.1))
builder.SetSection(msection)
builder.SetNumElements(5)
builder.SetYup(chrono.ChVector3d(0, 1, 0))  # Specify 'Y' up direction
builder.BuildBeam(mesh, sys)

# Fix the last node of the created beam
last_node = builder.GetLastBeamNodes().back()
last_node.SetFixed(True)

# Apply a force to the first node of the created beam section
first_node = builder.GetFirstBeamNodes().front()
first_node.SetForce(chrono.ChVector3d(0, -1, 0))

# Modify Existing Node-Fixing Approach
# =====================================
## Create a fixed truss
mtruss = chrono.ChBody()
mtruss.SetFixed(True)
sys.Add(mtruss)

## Create and initialize a constraint that connects node 1 to the fixed truss.
constr_d = chrono.ChLinkMateGeneric()
constr_d.Initialize(first_node, mtruss, False, first_node.Frame(), first_node.Frame())
sys.Add(constr_d)
constr_d.SetConstrainedCoords(False, True, True, False, False, False)

# Add visualization for the beams in the mesh.
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)

# Add visualization for the nodes in the mesh.
visualizebeamC = chrono.ChVisualShapeFEA(mesh)
visualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
visualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
visualizebeamC.SetSymbolsThickness(0.006)
visualizebeamC.SetSymbolsScale(0.01)
visualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualizebeamC)

# Create an Irrlicht visualization window.
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.1, 0.1, 0.2))
vis.AddTypicalLights()

# Change the default solver to the MKL Pardiso solver
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# Simulation loop.
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.001)