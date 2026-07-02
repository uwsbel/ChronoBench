import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

print("Example: PyChrono using beam finite elements")

# Create the physical system
sys = chrono.ChSystemSMC()

# Create a mesh for FEA elements
mesh = fea.ChMesh()

# Create a section for Euler-Bernoulli beam properties
msection = fea.ChBeamSectionEulerAdvanced()

# Set rectangular cross-section dimensions
beam_wy = 0.012
beam_wz = 0.025
msection.SetAsRectangularSection(beam_wy, beam_wz)

# Set material properties
msection.SetYoungModulus(0.01e9)
msection.SetShearModulus(0.01e9 * 0.3)
msection.SetRayleighDamping(0.000)
msection.SetCentroid(0, 0.02)
msection.SetShearCenter(0, 0.1)
msection.SetSectionRotation(45 * chrono.CH_DEG_TO_RAD)  # Corrected angle conversion

# Create original beam nodes using ChFrame
hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFrame(chrono.ChVector3d(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFrame(chrono.ChVector3d(0.1, 0, 0)))
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFrame(chrono.ChVector3d(0.2, 0, 0)))

# Add original nodes to mesh
mesh.AddNode(hnode1)
mesh.AddNode(hnode2)
mesh.AddNode(hnode3)

# Create original beam elements
belement1 = fea.ChElementBeamEuler()
belement1.SetNodes(hnode1, hnode2)
belement1.SetSection(msection)
mesh.AddElement(belement1)

belement2 = fea.ChElementBeamEuler()
belement2.SetNodes(hnode2, hnode3)
belement2.SetSection(msection)
mesh.AddElement(belement2)

# Apply force and torque
hnode2.SetForce(chrono.ChVector3d(4, 2, 0))
hnode3.SetTorque(chrono.ChVector3d(0, -0.04, 0))

# Create fixed truss body
mtruss = chrono.ChBody()
mtruss.SetFixed(True)
sys.Add(mtruss)

# Replace constraints with ChLinkNodeBody for node 1 and 3
constr_bc = fea.ChLinkNodeBody()
constr_bc.Initialize(hnode3, mtruss, chrono.ChFramed(hnode3.GetPos()))
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, True, True, True, True)

constr_d = fea.ChLinkNodeBody()
constr_d.Initialize(hnode1, mtruss, chrono.ChFramed(hnode1.GetPos()))
sys.Add(constr_d)
constr_d.SetConstrainedCoords(False, True, True, False, False, False)

# Create new beam using ChBuilderBeamEuler
builder = fea.ChBuilderBeamEuler()
builder.SetSection(msection)
builder.SetYDirection(chrono.ChVector3d(0, 1, 0))  # Y up direction
builder.SetLength(0.2)
builder.SetDirection(chrono.ChVector3d(1, 0, 0))  # Direction along x-axis
builder.BuildBeam(mesh, 5, chrono.ChVector3d(0, 0, -0.1))  # Start point

# Apply force to first node of new beam
nodes_new = builder.GetLastBeamNodes()
nodes_new[0].SetForce(chrono.ChVector3d(0, -1, 0))

# Fix last node of new beam
nodes_new[-1].SetFixed(True)

# Disable automatic gravity
mesh.SetAutomaticGravity(False)

# Add mesh to system
sys.Add(mesh)

# Add beam visualization
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)

# Add node visualization
visualizebeamC = chrono.ChVisualShapeFEA(mesh)
visualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
visualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
visualizebeamC.SetSymbolsThickness(0.006)
visualizebeamC.SetSymbolsScale(0.01)
visualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualizebeamC)

# Create visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.1, 0.1, 0.2))
vis.AddTypicalLights()

# Set MKL solver
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.001)