"""Euler-Bernoulli beam finite-element demo using the ChBuilderBeamEuler helper.

System type: ChSystemSMC (required for the stiff FEA beam stiffness matrices).
Main structure: a chain of Euler-Bernoulli beams. A first manual beam (three
ChNodeFEAxyzrot nodes / two ChElementBeamEuler elements) carries a nodal force and
torque; its end node is rigidly clamped and node 1 is fixed to ground through a
ChLinkMateGeneric constraint (instead of a direct SetFixed). A ChBuilderBeamEuler
helper then builds a 5-element beam from point A (0, 0, -0.1) to point B
(0.2, 0, -0.1) with a 'Y' up direction: its last node is fixed and a downward
force (0, -1, 0) N is applied to its first node. A second builder beam continues
from that node.

Expected behavior: the constrained/loaded beams deflect and settle into their
static bending shapes, visualized by the Mz bending-moment colour field. The MKL
Pardiso direct solver integrates the stiff beam dynamics. World is Y-up; FEA
gravity is disabled (static/forced response).
"""

# === Imports ===
import os
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / section / run control
BEAM_L = 0.1                                # manual-beam element length [m]
BEAM_WY = 0.012                             # rectangular section width y [m]
BEAM_WZ = 0.025                             # rectangular section width z [m]
YOUNG = 0.01e9                              # Young's modulus [Pa]
SHEAR = 0.01e9 * 0.3                        # shear modulus [Pa]

BUILDER_A = chrono.ChVector3d(0, 0, -0.1)   # builder beam start point A
BUILDER_B = chrono.ChVector3d(0.2, 0, -0.1) # builder beam end point B
BUILDER_N = 5                               # number of builder beam elements
UP_Y = chrono.ChVector3d(0, 1, 0)           # 'Y' up direction of the section

TIME_STEP = 1e-3                            # stiff-beam timestep
SIM_END = 5.0                               # simulation end time [s]
RENDER_FPS = 50.0                           # review render cadence
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once


# === System & gravity === SMC system, Y-up world for FEA beams
sys = chrono.ChSystemSMC()

# === FEA mesh & beam section ===
# Strong refs kept (SWIG GC pitfall): mesh, section, nodes, elements, builder, truss.
mesh = fea.ChMesh()

# Shared Euler-Bernoulli section (rectangular).
msection = fea.ChBeamSectionEulerAdvanced()
msection.SetAsRectangularSection(BEAM_WY, BEAM_WZ)
msection.SetYoungModulus(YOUNG)
msection.SetShearModulus(SHEAR)
msection.SetRayleighDamping(0.000)

# --- Manual Euler-Bernoulli beam: three nodes, two elements ---
hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(BEAM_L, 0, 0)))
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(BEAM_L * 2, 0, 0)))
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

# Apply a force and a torque to nodes of the manual beam.
hnode2.SetForce(chrono.ChVector3d(4, 2, 0))
hnode3.SetTorque(chrono.ChVector3d(0, -0.04, 0))

# === Joints / constraints ===
# (FEA beams: no contact material needed — driven by constraints + nodal loads only.)
# Fix node 1 using constraints (instead of a direct hnode1.SetFixed(True)).
mtruss = chrono.ChBody()
mtruss.SetFixed(True)
sys.Add(mtruss)

# Clamp the far end node (hnode3) in all 6 DOF.
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(hnode3, mtruss, False, hnode3.Frame(), hnode3.Frame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, True,    # x, y, z
                               True, True, True)    # Rx, Ry, Rz

# Fix node 1 to ground via a ChLinkMateGeneric constraint (selected DOF).
constr_d = chrono.ChLinkMateGeneric()
constr_d.Initialize(hnode1, mtruss, False, hnode1.Frame(), hnode1.Frame())
sys.Add(constr_d)
constr_d.SetConstrainedCoords(False, True, True,    # x, y, z
                              False, False, False)  # Rx, Ry, Rz

# --- ChBuilderBeamEuler: 5-element beam from point A to point B ('Y' up) ---
builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(mesh, msection, BUILDER_N, BUILDER_A, BUILDER_B, UP_Y)

# Fix the last node of the created beam; apply a force to the first node.
builder.GetLastBeamNodes().back().SetFixed(True)
builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -1, 0))

# Continue with a second beam from the last builder front node to a new point.
builder.BuildBeam(mesh, msection, BUILDER_N,
                  builder.GetLastBeamNodes().front(),
                  chrono.ChVector3d(0.2, 0.1, -0.1),
                  UP_Y)

# No gravity effect on the FEA elements in this demo.
mesh.SetAutomaticGravity(False)
sys.Add(mesh)

# === Solver === MKL Pardiso direct solver (precise for FEA)
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# === FEA visualization === Mz bending-moment surface + node-CSYS glyphs
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

# === Visualization === full Irrlicht scene: window + sky + camera + lights
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("FEA beams")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.1, 0.1, 0.2))
vis.AddTypicalLights()

# === Main loop === render-cadence outer loop; physics in the inner batch

frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
