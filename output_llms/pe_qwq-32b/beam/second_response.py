import pychrono as chrono  # Import the main PyChrono module.
import pychrono.fea as fea  # Import the finite element analysis (FEA) module.
import pychrono.pardisomkl as mkl  # Import the Pardiso MKL linear solver module.
import pychrono.irrlicht as chronoirr  # Import the Irrlicht visualization module.

print("Example: PyChrono using beam finite elements")  # Print an introductory statement.

# Create the physical system that will be simulated.
sys = chrono.ChSystemSMC()

# Create a mesh, which is a container for elements and their referenced nodes.
mesh = fea.ChMesh()

# Create a section object for beam properties.
msection = fea.ChBeamSectionEulerAdvanced()

# Define beam cross-section properties
beam_wy = 0.012
beam_wz = 0.025
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(0.01e9)
msection.SetShearModulus(0.01e9 * 0.3)
msection.SetRayleighDamping(0.000)
msection.SetCentroid(0, 0.02)
msection.SetShearCenter(0, 0.1)
msection.SetSectionRotation(45 * chrono.CH_DEG_TO_RAD)  # Correct angle conversion to radians

beam_L = 0.1

# Create nodes for the original beams
hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVectorD(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVectorD(beam_L, 0, 0)))
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVectorD(beam_L * 2, 0, 0)))

# Add nodes to the mesh
mesh.AddNode(hnode1)
mesh.AddNode(hnode2)
mesh.AddNode(hnode3)

# Create original beams
belement1 = fea.ChElementBeamEuler()
belement1.SetNodes(hnode1, hnode2)
belement1.SetSection(msection)
mesh.AddElement(belement1)

belement2 = fea.ChElementBeamEuler()
belement2.SetNodes(hnode2, hnode3)
belement2.SetSection(msection)
mesh.AddElement(belement2)

# Apply forces/torques to original nodes
hnode2.SetForce(chrono.ChVectorD(4, 2, 0))
hnode3.SetTorque(chrono.ChVectorD(0, -0.04, 0))

# Create fixed truss
mtruss = chrono.ChBody()
mtruss.SetFixed(True)
sys.Add(mtruss)

# Correct constraints for original nodes
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(hnode3, mtruss, True, hnode3.Frame(), hnode3.Frame())  # Fixed node3
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, True, True, True, True)

constr_d = chrono.ChLinkMateGeneric()
constr_d.Initialize(hnode1, mtruss, True, hnode1.Frame(), hnode1.Frame())  # Fixed node1's y,z
sys.Add(constr_d)
constr_d.SetConstrainedCoords(False, True, True, False, False, False)

# --- New Euler-Bernoulli beam using builder ---
# Create a new beam using ChBuilderBeamEuler helper
builder = fea.ChBuilderBeamEuler(mesh)
builder.SetSection(msection)

# Create beam from (0,0,-0.1) to (0.2,0,-0.1) with Y up and 5 elements
builder.BuildBeam(
    chrono.ChVectorD(0, 0, -0.1),
    chrono.ChVectorD(0.2, 0, -0.1),
    5,
    chrono.ChVectorD(0, 1, 0)  # Y-axis as up direction
)

# Fix last node of the new beam
nodes = builder.GetLastBeamNodes()
last_node = nodes[-1]
last_node.SetFixed(True)

# Apply force to first node of new beam
first_node = nodes[0]
first_node.SetForce(chrono.ChVectorD(0, -1, 0))

# --- Rest of the original code ---
mesh.SetAutomaticGravity(False)
sys.Add(mesh)

# Visualization for original beams
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)

visualizebeamC = chrono.ChVisualShapeFEA(mesh)
visualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
visualizebeamC.SetSymbolsThickness(0.006)
visualizebeamC.SetSymbolsScale(0.01)
visualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualizebeamC)

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.1, 0.1, 0.2))
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