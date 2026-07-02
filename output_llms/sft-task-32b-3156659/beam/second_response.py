import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

print("Example: PyChrono using beam finite elements")

# Create the physical system
sys = chrono.ChSystemSMC()

# Create a mesh for FEA elements
mesh = fea.ChMesh()

# Create a beam section with Euler-Bernoulli properties
msection = fea.ChBeamSectionEulerAdvanced()

# Set rectangular cross-section dimensions (width in Y, height in Z)
beam_wy = 0.012
beam_wz = 0.025
msection.SetAsRectangularSection(beam_wy, beam_wz)

# Set material properties
msection.SetYoungModulus(0.01e9)  # Young's modulus
msection.SetShearModulus(0.01e9 * 0.3)  # Shear modulus
msection.SetRayleighDamping(0.000)  # Damping coefficient

# Correct centroid and shear center positions for rectangular section
msection.SetCentroid(0, beam_wz / 2)  # Centroid at center of height
msection.SetShearCenter(0, beam_wz / 2)  # Shear center same as centroid for rectangular section

# Correct section rotation angle (45 degrees in radians)
msection.SetSectionRotation(45 * chrono.CH_DEG_TO_RAD)

# Create beam using ChBuilderBeamEuler helper
builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(
    mesh,  # mesh where to put the created elements
    5,     # number of elements
    0.2,   # length of the beam
    msection,  # beam section properties
    chrono.ChVector3d(0, 0, -0.1),  # start point
    chrono.ChVector3d(0.2, 0, -0.1),  # end point
    chrono.ChVector3d(0, 1, 0)  # Y-up direction
)

# Fix the last node of the beam
nodes = builder.GetLastBeamNodes()
nodes[-1].SetFixed(True)

# Apply force to the first node of the beam
nodes[0].SetForce(chrono.ChVector3d(0, -1, 0))

# Create a fixed truss body
mtruss = chrono.ChBody()
mtruss.SetFixed(True)
sys.Add(mtruss)

# Create constraint to fix first node's Y and Z translations
constr_d = chrono.ChLinkMateGeneric()
constr_d.Initialize(nodes[0], mtruss, False, nodes[0].Frame(), nodes[0].Frame())
sys.Add(constr_d)
constr_d.SetConstrainedCoords(False, True, True, False, False, False)  # Fix Y and Z translations

# Disable automatic gravity for FEA elements
mesh.SetAutomaticGravity(False)

# Add mesh to the system
sys.Add(mesh)

# Add visualization for beam bending moments
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)

# Add visualization for node coordinate systems
visualizebeamC = chrono.ChVisualShapeFEA(mesh)
visualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
visualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
visualizebeamC.SetSymbolsThickness(0.006)
visualizebeamC.SetSymbolsScale(0.01)
visualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualizebeamC)

# Create Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.1, 0.1, 0.2))
vis.AddTypicalLights()

# Set up MKL Pardiso solver
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.001)