import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

print("Example: PyChrono using beam finite elements")

sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# 1. Euler-Bernoulli beam section setup
msection = fea.ChBeamSectionEulerAdvanced()
beam_wy = 0.012
beam_wz = 0.025
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(0.01e9)
msection.SetShearModulus(0.01e9 * 0.3)
msection.SetRayleighDamping(0.000)
msection.SetCentroid(0, 0.02)
msection.SetShearCenter(0, 0.1)
msection.SetSectionRotation(45 * chrono.CH_DEG_TO_RAD)  # Corrected angle conversion

# Original beam nodes with corrected ChFrameD
beam_L = 0.1
hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVector3d(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVector3d(beam_L, 0, 0)))
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVector3d(beam_L * 2, 0, 0)))

mesh.AddNode(hnode1)
mesh.AddNode(hnode2)
mesh.AddNode(hnode3)

# Original beam elements
belement1 = fea.ChElementBeamEuler()
belement1.SetNodes(hnode1, hnode2)
belement1.SetSection(msection)
mesh.AddElement(belement1)

belement2 = fea.ChElementBeamEuler()
belement2.SetNodes(hnode2, hnode3)
belement2.SetSection(msection)
mesh.AddElement(belement2)

# Applied forces
hnode2.SetForce(chrono.ChVector3d(4, 2, 0))
hnode3.SetTorque(chrono.ChVector3d(0, -0.04, 0))

# 2. Modified constraints using proper FEA constraints
mtruss = chrono.ChBody()
mtruss.SetFixed(True)
sys.Add(mtruss)

# Constraint for node3 (full fixity)
constr_node3 = fea.ChLinkNodeFrame()
constr_node3.Initialize(hnode3, mtruss)
sys.Add(constr_node3)

# Constraint for node1 (y and z translations fixed)
constr_node1 = fea.ChLinkNodeFrame()
constr_node1.Initialize(hnode1, mtruss)
sys.Add(constr_node1)

# 3. Create new beam using ChBuilderBeamEuler
builder = fea.ChBuilderBeamEuler()
start_point = chrono.ChVector3d(0, 0, -0.1)
end_point = chrono.ChVector3d(0.2, 0, -0.1)
builder.BuildBeam(mesh, msection, 5, start_point, end_point, chrono.ChVector3d(0, 1, 0))

# Fix last node and apply force to first node of new beam
builder.GetLastBeamNodes().back().SetFixed(True)
builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -1, 0))

mesh.SetAutomaticGravity(False)
sys.Add(mesh)

# 4. Maintain visualization settings
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

# Visualization system
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