import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

print("Example: PyChrono using beam finite elements")

# -----------------------------------------------------------------
# 1.  Create the mechanical system and a mesh container
# -----------------------------------------------------------------
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# -----------------------------------------------------------------
# 2.  Define a beam section (thickness & material)
# -----------------------------------------------------------------
msection = fea.ChBeamSectionEulerAdvanced()
beam_wy = 0.012
beam_wz = 0.025
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(0.01e9)
msection.SetShearModulus(0.01e9 * 0.3)
msection.SetRayleighDamping(0.000)

#  *** correction: SetCentroid / SetShearCenter expect a ChVector3d ***
msection.SetCentroid(chrono.ChVector3d(0, 0.02, 0))
msection.SetShearCenter(chrono.ChVector3d(0, 0.1, 0))

#  *** correction: rotation angle must be in radians; use CH_DEG_TO_RAD ***
msection.SetSectionRotation(45 * chrono.CH_DEG_TO_RAD)

# -----------------------------------------------------------------
# 3.  Manually add a few Euler‑Bernoulli beam elements
# -----------------------------------------------------------------
beam_L = 0.1

#  *** correction: use chrono.ChFrame (not chrono.ChFramed) ***
hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFrame(chrono.ChVector3d(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFrame(chrono.ChVector3d(beam_L, 0, 0)))
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFrame(chrono.ChVector3d(beam_L * 2, 0, 0)))

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

# -----------------------------------------------------------------
# 4.  Apply nodal forces / torques
# -----------------------------------------------------------------
hnode2.SetForce(chrono.ChVector3d(4, 2, 0))
hnode3.SetTorque(chrono.ChVector3d(0, -0.04, 0))

# -----------------------------------------------------------------
# 5.  Fix some nodes using constraints (a dummy truss body)
# -----------------------------------------------------------------
mtruss = chrono.ChBody()
mtruss.SetFixed(True)
sys.Add(mtruss)

# Fully fix hnode3
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(hnode3, mtruss, False, hnode3.Frame(), hnode3.Frame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, True, True, True, True)

# Partially fix hnode1 (allow only x‑translation)
constr_d = chrono.ChLinkMateGeneric()
constr_d.Initialize(hnode1, mtruss, False, hnode1.Frame(), hnode1.Frame())
sys.Add(constr_d)
constr_d.SetConstrainedCoords(False, True, True, False, False, False)

# -----------------------------------------------------------------
# 6.  Build a beam with the fast builder (first segment)
# -----------------------------------------------------------------
builder = fea.ChBuilderBeamEuler()

builder.BuildBeam(
    mesh,
    msection,
    5,
    chrono.ChVector3d(0, 0, -0.1),          # A point
    chrono.ChVector3d(0.2, 0, -0.1),       # B point
    chrono.ChVector3d(0, 1, 0)             # Y‑up direction
)

# Fix the end of this beam and apply a force at its start
builder.GetLastBeamNodes().back().SetFixed(True)
builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -1, 0))

# -----------------------------------------------------------------
# 7.  Add the second beam segment – start = last node of previous beam
# -----------------------------------------------------------------
#  <<<<  NEW BEAM SEGMENT >>>>
last_node_prev = builder.GetLastBeamNodes().back()            # node at (0.2,0,-0.1)
start_pos = last_node_prev.Frame().GetPos()                   # its position

builder.BuildBeam(
    mesh,
    msection,
    5,
    start_pos,                                 # A = last node of previous beam
    chrono.ChVector3d(0.2, 0.1, -0.1),          # B = new endpoint
    chrono.ChVector3d(0, 1, 0)                  # Y‑up direction
)

# Optional: fix the new beam’s end and load its start
builder.GetLastBeamNodes().back().SetFixed(True)
builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.5, 0))
# -----------------------------------------------------------------
# 8.  Disable gravity on the FEA mesh and add it to the system
# -----------------------------------------------------------------
mesh.SetAutomaticGravity(False)
sys.Add(mesh)

# -----------------------------------------------------------------
# 9.  Visualisation assets
# -----------------------------------------------------------------
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

# -----------------------------------------------------------------
# 10.  Irrlicht visualisation
# -----------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA beams')
vis.Initialize()

# Logo – skip if file is missing
try:
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
except Exception:
    pass

vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.1, 0.1, 0.2))
vis.AddTypicalLights()

# -----------------------------------------------------------------
# 11.  Solver (use MKL Pardiso for better FEA performance)
# -----------------------------------------------------------------
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# -----------------------------------------------------------------
# 12.  Simulation loop
# -----------------------------------------------------------------
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.001)