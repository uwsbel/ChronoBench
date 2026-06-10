"""
Buckling demo — PyChrono FEA with Euler-Bernoulli beam elements.

Models a classic column-buckling mechanism: a fixed truss, a horizontal Euler
beam spanning from the truss, a vertical Euler column whose base is fixed to
the truss, and a crank driven by a ChLinkMotorRotationSpeed. The crank beam
connects the rotating crank body to the top node of the vertical column.
As the motor rotates the crank, it applies a transverse load to the top of the
vertical column, inducing buckling. Uses ChSystemSMC + Pardiso MKL direct solver
+ HHT timestepper. Y-up world convention.

Turn-2 geometry: L=1.2, H=0.3, K=0.07; horizontal beam 0.12x0.012 rectangular,
vertical column circular d=0.03 with 6 Euler elements, crank beam circular
d=0.054 with 5 Euler elements. Constraint sphere markers 0.012 and 0.014.
Glyph scale 0.015. Camera at (0.0, 0.7, -1.2).
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl


# === Geometry & physics constants ===
L = 1.2          # length scale (horizontal beam length)
H = 0.3          # height (vertical beam length)
K = 0.07         # crank arm length

# Beam cross-sections
HORIZ_WY = 0.12       # horizontal beam Y width (rectangular)
HORIZ_WZ = 0.012      # horizontal beam Z width (rectangular)
VERT_DIAM = 0.03      # vertical column circular diameter
CRANK_DIAM = 0.054    # crank beam circular diameter
N_VERT_ELEMS = 6      # number of Euler elements in vertical beam
N_CRANK_ELEMS = 5     # number of Euler elements in crank beam

# Visualization sphere sizes for constraint markers
CONSTR_SPHERE_1 = 0.012   # sphere at truss-to-horizontal-beam constraint
CONSTR_SPHERE_2 = 0.014   # sphere at crank-beam-to-vertical-beam constraint

GLYPH_SCALE = 0.015   # scale for FEA glyph visualization

# Material (steel)
DENSITY = 7800.0
YOUNG_MOD = 210e9
POISSON = 0.3
RAYLEIGH_DAMP = 0.0002

# Simulation
TIME_STEP = 0.001
SIM_END = 10.0
RENDER_FPS = 50.0

# === System & gravity (Y-up FEA scene; ChSystemSMC + Pardiso MKL + HHT) ===
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Pardiso MKL direct solver — required for stiff beam stiffness matrices
sys.SetSolver(mkl.ChSolverPardisoMKL())

# HHT timestepper — canonical-minimal form (exactly two calls per ground truth)
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# FEA beam: no contact material needed — driven by constraints + motor only

# === Bodies: truss (fixed support) and crank arm ===
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetName("truss")
truss.SetPos(chrono.ChVector3d(0, 0, 0))
# Truss visualization box: 0.03 x 0.25 x 0.12
truss_shape = chrono.ChVisualShapeBox(0.03, 0.25, 0.12)
truss.AddVisualShape(truss_shape)
sys.Add(truss)

# Crank body — pivot at (0, 0, 0) on truss, arm extends to K in X
crank = chrono.ChBody()
crank.SetName("crank")
crank.SetPos(chrono.ChVector3d(K * 0.5, 0, 0))  # COM at midpoint of crank arm
# Crank visualization box: K x 0.03 x 0.03
crank_shape = chrono.ChVisualShapeBox(K, 0.03, 0.03)
crank.AddVisualShape(crank_shape)
sys.Add(crank)

# === Motor: spin crank relative to truss around Z-axis at origin ===
motor = chrono.ChLinkMotorRotationSpeed()
motor.SetName("motor_crank")
motor.Initialize(
    crank,
    truss,
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT)
)
motor_speed_fn = chrono.ChFunctionConst(-0.5)  # 0.5 rad/s, slow crank rotation
motor.SetMotorFunction(motor_speed_fn)
sys.Add(motor)

# === FEA meshes ===
mesh_h = fea.ChMesh()   # horizontal Euler beam (along X, fixed to truss wall)
mesh_h.SetAutomaticGravity(False)

mesh_v = fea.ChMesh()   # vertical Euler column (along Y, to be buckled)
mesh_v.SetAutomaticGravity(True)

mesh_c = fea.ChMesh()   # crank beam (connecting crank body to column top)
mesh_c.SetAutomaticGravity(False)

# --- Horizontal beam: rectangular section, Euler-Bernoulli, 4 elements ---
# Runs from x=0 to x=L at y=0 (along the truss wall)
sec_h = fea.ChBeamSectionEulerAdvanced()
sec_h.SetAsRectangularSection(HORIZ_WY, HORIZ_WZ)
sec_h.SetDensity(DENSITY)
sec_h.SetYoungModulus(YOUNG_MOD)
sec_h.SetShearModulusFromPoisson(POISSON)
sec_h.SetRayleighDamping(RAYLEIGH_DAMP)

builder_h = fea.ChBuilderBeamEuler()
builder_h.BuildBeam(
    mesh_h, sec_h, 4,
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(L, 0, 0),
    chrono.VECT_Y
)
# Keep strong ref to node container (SWIG GC pitfall)
nodes_h_ref = builder_h.GetLastBeamNodes()
hbeam_nodes = [nodes_h_ref[i] for i in range(nodes_h_ref.size())]

# Fix start node of horizontal beam to truss (6 DOF)
constr_h_fix = chrono.ChLinkMateGeneric()
constr_h_fix.Initialize(hbeam_nodes[0], truss, False,
                         hbeam_nodes[0].Frame(), hbeam_nodes[0].Frame())
constr_h_fix.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(constr_h_fix)

sys.Add(mesh_h)

# --- Vertical column: circular section, Euler-Bernoulli, N_VERT_ELEMS elements ---
# Runs from y=0 to y=H at x=L (the column to be buckled)
sec_v = fea.ChBeamSectionEulerAdvanced()
sec_v.SetAsCircularSection(VERT_DIAM)
sec_v.SetDensity(DENSITY)
sec_v.SetYoungModulus(YOUNG_MOD)
sec_v.SetShearModulusFromPoisson(POISSON)
sec_v.SetRayleighDamping(RAYLEIGH_DAMP)

builder_v = fea.ChBuilderBeamEuler()
builder_v.BuildBeam(
    mesh_v, sec_v, N_VERT_ELEMS,
    chrono.ChVector3d(L, 0, 0),
    chrono.ChVector3d(L, H, 0),
    chrono.VECT_X
)
# Keep strong ref to node container (SWIG GC pitfall)
nodes_v_ref = builder_v.GetLastBeamNodes()
vbeam_nodes = [nodes_v_ref[i] for i in range(nodes_v_ref.size())]

# Fix base of vertical column to truss (6 DOF)
constr_v_fix = chrono.ChLinkMateGeneric()
constr_v_fix.Initialize(vbeam_nodes[0], truss, False,
                         vbeam_nodes[0].Frame(), vbeam_nodes[0].Frame())
constr_v_fix.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(constr_v_fix)

sys.Add(mesh_v)

# --- Crank beam: circular section, Euler-Bernoulli, N_CRANK_ELEMS elements ---
# Runs from (K, 0, 0) [tip of crank arm] to the top node of the vertical column (L, H, 0)
sec_c = fea.ChBeamSectionEulerAdvanced()
sec_c.SetAsCircularSection(CRANK_DIAM)
sec_c.SetDensity(DENSITY)
sec_c.SetYoungModulus(YOUNG_MOD)
sec_c.SetShearModulusFromPoisson(POISSON)
sec_c.SetRayleighDamping(RAYLEIGH_DAMP)

builder_c = fea.ChBuilderBeamEuler()
builder_c.BuildBeam(
    mesh_c, sec_c, N_CRANK_ELEMS,
    chrono.ChVector3d(K, 0, 0),
    chrono.ChVector3d(L, H, 0),
    chrono.VECT_Z
)
# Keep strong ref to node container (SWIG GC pitfall)
nodes_c_ref = builder_c.GetLastBeamNodes()
cbeam_nodes = [nodes_c_ref[i] for i in range(nodes_c_ref.size())]

# Attach crank beam start node to crank body tip (at position K, 0, 0)
constr_c_to_crank = chrono.ChLinkMateGeneric()
constr_c_to_crank.Initialize(
    cbeam_nodes[0], crank, False,
    cbeam_nodes[0].Frame(), cbeam_nodes[0].Frame()
)
constr_c_to_crank.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(constr_c_to_crank)

# Attach crank beam end node to the top node of the vertical column (shared pin)
constr_c_to_vtop = chrono.ChLinkMateGeneric()
constr_c_to_vtop.Initialize(
    cbeam_nodes[-1], vbeam_nodes[-1], False,
    cbeam_nodes[-1].Frame(), vbeam_nodes[-1].Frame()
)
constr_c_to_vtop.SetConstrainedCoords(True, True, True, False, False, False)
sys.Add(constr_c_to_vtop)

sys.Add(mesh_c)

# === FEA Visualization — two-shape pattern: surface + glyphs per mesh ===

# Horizontal beam: moment field + glyphs
vis_h_surf = chrono.ChVisualShapeFEA(mesh_h)
vis_h_surf.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_h_surf.SetColorscaleMinMax(-0.4, 0.4)
vis_h_surf.SetSmoothFaces(True)
vis_h_surf.SetWireframe(False)
mesh_h.AddVisualShapeFEA(vis_h_surf)

vis_h_glyph = chrono.ChVisualShapeFEA(mesh_h)
vis_h_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_h_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_h_glyph.SetSymbolsThickness(0.006)
vis_h_glyph.SetSymbolsScale(GLYPH_SCALE)
vis_h_glyph.SetZbufferHide(False)
mesh_h.AddVisualShapeFEA(vis_h_glyph)

# Vertical column: moment field + glyphs
vis_v_surf = chrono.ChVisualShapeFEA(mesh_v)
vis_v_surf.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_v_surf.SetColorscaleMinMax(-0.4, 0.4)
vis_v_surf.SetSmoothFaces(True)
vis_v_surf.SetWireframe(False)
mesh_v.AddVisualShapeFEA(vis_v_surf)

vis_v_glyph = chrono.ChVisualShapeFEA(mesh_v)
vis_v_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_v_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_v_glyph.SetSymbolsThickness(0.006)
vis_v_glyph.SetSymbolsScale(GLYPH_SCALE)
vis_v_glyph.SetZbufferHide(False)
mesh_v.AddVisualShapeFEA(vis_v_glyph)

# Crank beam: moment field + glyphs
vis_c_surf = chrono.ChVisualShapeFEA(mesh_c)
vis_c_surf.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_c_surf.SetColorscaleMinMax(-0.4, 0.4)
vis_c_surf.SetSmoothFaces(True)
vis_c_surf.SetWireframe(False)
mesh_c.AddVisualShapeFEA(vis_c_surf)

vis_c_glyph = chrono.ChVisualShapeFEA(mesh_c)
vis_c_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_c_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_c_glyph.SetSymbolsThickness(0.006)
vis_c_glyph.SetSymbolsScale(GLYPH_SCALE)
vis_c_glyph.SetZbufferHide(False)
mesh_c.AddVisualShapeFEA(vis_c_glyph)

# === Constraint visualization sphere markers ===
# Sphere at truss-to-beam constraint (size 0.012)
sphere_body_1 = chrono.ChBody()
sphere_body_1.SetFixed(True)
sphere_body_1.SetPos(hbeam_nodes[0].GetPos())
sphere_body_1.AddVisualShape(chrono.ChVisualShapeSphere(CONSTR_SPHERE_1))
sys.Add(sphere_body_1)

# Sphere at crank-beam-to-vertical-column connection (size 0.014)
sphere_body_2 = chrono.ChBody()
sphere_body_2.SetFixed(True)
sphere_body_2.SetPos(vbeam_nodes[-1].GetPos())
sphere_body_2.AddVisualShape(chrono.ChVisualShapeSphere(CONSTR_SPHERE_2))
sys.Add(sphere_body_2)

# === Visualization (Irrlicht — Initialize FIRST, then scene elements AFTER) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono FEA Buckling Demo")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
# Updated camera position per turn-2 spec
vis.AddCamera(chrono.ChVector3d(0.0, 0.7, -1.2), chrono.ChVector3d(0.5 * L, 0.3 * H, 0.0))
vis.AddTypicalLights()

# === Main loop ===
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once


try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
