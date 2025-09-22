#
#  PyChrono demo:  FEA beams – Euler-Bernoulli example
#
#  This script shows how to build beams with two different approaches:
#    1) “manual” creation of nodes/elements;
#    2) use of the ChBuilderBeamEuler helper (Euler-Bernoulli formulation).
#
#  The original code supplied in the exercise has been
#  (i) checked for API mistakes, (ii) cleaned-up, and
#  (iii) extended to satisfy the requested modifications.
#

# =============================================================================
# imports
# =============================================================================
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr
import math

print("Example: PyChrono using beam finite elements")

# =============================================================================
# basic system
# =============================================================================
sys = chrono.ChSystemSMC()

# -----------------------------------------------------------------------------
# a mesh that will contain both the manually-made beam elements and the builder
# generated Euler-Bernoulli beam
# -----------------------------------------------------------------------------
mesh = fea.ChMesh()

# =============================================================================
# SECTION definition (common to all beams in this example)
# =============================================================================
msection = fea.ChBeamSectionEulerAdvanced()

beam_wy = 0.012
beam_wz = 0.025
msection.SetAsRectangularSection(beam_wy, beam_wz)

msection.SetYoungModulus(0.01e9)
msection.SetShearModulus(0.01e9 * 0.3)
msection.SetRayleighDamping(0.0)
msection.SetCentroid(0, 0.02)
msection.SetShearCenter(0, 0.1)

# the API expects radians:
msection.SetSectionRotation(45 * chrono.CH_DEG_TO_RAD)

# =============================================================================
# --- 1) “manual” creation of a short 2-element beam --------------------------
# =============================================================================
beam_L = 0.1

hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(beam_L, 0, 0)))
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(2 * beam_L, 0, 0)))

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

# external loads
hnode2.SetForce(chrono.ChVectorD(4, 2, 0))
hnode3.SetTorque(chrono.ChVectorD(0, -0.04, 0))

# -----------------------------------------------------------------------------
# fixing node-3 completely to ground and replacing the “fixed” flag on node-1
# with a constraint (requested modification #2).
# -----------------------------------------------------------------------------
mtruss = chrono.ChBody()
mtruss.SetFixed(True)
sys.Add(mtruss)

# … node-3 fully fixed
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(hnode3, mtruss, False,
                     hnode3.Frame(), hnode3.Frame())
constr_bc.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(constr_bc)

# … node-1 fixed via constraints (instead of SetFixed)
# hnode1.SetFixed(True)    # <-- original direct fix – now commented out
constr_a = chrono.ChLinkMateGeneric()
constr_a.Initialize(hnode1, mtruss, False,
                    hnode1.Frame(), hnode1.Frame())
constr_a.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(constr_a)

# =============================================================================
# --- 2) Euler-Bernoulli beam built with ChBuilderBeamEuler -------------------
# =============================================================================
# The following block addresses requirements 1 & 3 of the assignment.
# We build a beam that spans from A(0, 0, −0.1) to B(0.2, 0, −0.1) using
# the builder helper.  Five elements will be created with Y as the “up” vector.
# =============================================================================
builder = fea.ChBuilderBeamEuler()

point_A = chrono.ChVectorD(0, 0, -0.1)
point_B = chrono.ChVectorD(0.2, 0, -0.1)
up_dir  = chrono.ChVectorD(0, 1, 0)          # ‘Y’ up

# BuildBeam(mesh, section, n_elements, start, end, up_dir)
builder.BuildBeam(mesh,                       # mesh to populate
                  msection,                   # section previously defined
                  5,                          # number of elements
                  point_A,
                  point_B,
                  up_dir)

# Fix the last node of the new beam
builder.GetLastBeamNodes()[-1].SetFixed(True)

# Apply a downward force on the first node
builder.GetLastBeamNodes()[0].SetForce(chrono.ChVectorD(0, -1, 0))

# =============================================================================
# mesh & system finalisation
# =============================================================================
mesh.SetAutomaticGravity(False)
sys.Add(mesh)

# =============================================================================
# visualisation assets (unchanged apart from fea namespace correction)
# =============================================================================
visualizebeamA = fea.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)

visualizebeamC = fea.ChVisualShapeFEA(mesh)
visualizebeamC.SetFEMglyphType(fea.ChVisualShapeFEA.GlyphType_NODE_CSYS)
visualizebeamC.SetFEMdataType(fea.ChVisualShapeFEA.DataType_NONE)
visualizebeamC.SetSymbolsThickness(0.006)
visualizebeamC.SetSymbolsScale(0.01)
visualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualizebeamC)

# =============================================================================
# Irrlicht window
# =============================================================================
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA beams – Euler-Bernoulli example')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.1, 0.1, 0.2))
vis.AddTypicalLights()

# =============================================================================
# MKL Pardiso solver (kept as requested)
# =============================================================================
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# =============================================================================
# simulation loop
# =============================================================================
time_step = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(time_step)