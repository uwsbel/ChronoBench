import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

print("Example: PyChrono using beam finite elements")

# Create the physical system that will be simulated.
sys = chrono.ChSystemSMC()

# Create a mesh, which is a container for elements and their referenced nodes.
mesh = fea.ChMesh()

# =============================================
# 1. Add a Section on Euler-Bernoulli Beams
# =============================================
# Create a beam section using ChBuilderBeamEuler
builder = fea.ChBuilderBeamEuler()
builder.SetBeamSection(fea.ChBeamSectionEulerAdvanced())
builder.GetLastBeamSection().SetAsRectangularSection(0.012, 0.025)
builder.GetLastBeamSection().SetYoungModulus(0.01e9)
builder.GetLastBeamSection().SetShearModulus(0.01e9 * 0.3)
builder.GetLastBeamSection().SetRayleighDamping(0.000)
builder.GetLastBeamSection().SetCentroid(0, 0.02)
builder.GetLastBeamSection().SetShearCenter(0, 0.1)
builder.GetLastBeamSection().SetSectionRotation(45 * chrono.CH_RAD_TO_DEG)

# Build a beam from point A to point B with 5 elements
beam = builder.BuildBeam(mesh,
                        chrono.ChVector3d(0, 0, -0.1),
                        chrono.ChVector3d(0.2, 0, -0.1),
                        5,
                        chrono.ChVector3d(0, 1, 0))  # Y up direction

# =============================================
# 2. Modify Existing Node-Fixing Approach
# =============================================
# Fix the last node of the created beam
last_node = builder.GetLastBeamNodes().back()
last_node.SetFixed(True)

# Apply a force to the first node of the created beam
first_node = builder.GetLastBeamNodes().front()
first_node.SetForce(chrono.ChVector3d(0, -1, 0))

# =============================================
# 3. Use Builder Helper Object
# =============================================
# The beam was already created using the builder above

# Create a fixed truss for additional constraints
mtruss = chrono.ChBody()
mtruss.SetFixed(True)
sys.Add(mtruss)

# Create and initialize a constraint that connects the first node to the fixed truss
constr_d = chrono.ChLinkMateGeneric()
constr_d.Initialize(first_node, mtruss, False, first_node.Frame(), first_node.Frame())
sys.Add(constr_d)
constr_d.SetConstrainedCoords(False, True, True,  # Constrain only y, z translations
                              False, False, False)  # Do not constrain any rotations

# Disable the automatic gravity for FEA elements in this demonstration.
mesh.SetAutomaticGravity(False)

# Add the mesh to the physical system.
sys.Add(mesh)

# =============================================
# 4. Maintain Visualization and Solvers
# =============================================
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