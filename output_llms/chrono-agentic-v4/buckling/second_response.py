"""
Buckling FEA simulation — Euler beam column under compressive load.
System: ChSystemSMC with Pardiso MKL solver and HHT timestepper.
Bodies: vertical cantilever beam, horizontal truss, crank mechanism.
"""

import os
import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# === Geometry Parameters (turn 2 updates applied) ===
L = 1.2          # vertical beam length (updated from 1.0)
H = 0.3          # horizontal truss height (updated from 0.25)
K = 0.07         # crank length (updated from 0.05)
W_SX = 0.03      # truss visual width x
W_SY = 0.25      # truss visual width y (updated from 0.2)
W_SZ = 0.12      # truss visual depth z (updated from 0.1)
C_SX = K         # crank visual width x (= K)
C_SY = 0.03      # crank visual width y (updated from 0.02)
C_SZ = 0.03      # crank visual depth z (updated from 0.02)
HB_WY = 0.12     # horizontal beam width Y (updated from 0.10)
HB_WZ = 0.012    # horizontal beam width Z (updated from 0.01)
VB_DIAM = 0.03   # vertical beam circular section diameter (updated from 0.024)
VB_NEL = 6       # vertical beam Euler elements (updated from 3)
CB_DIAM = 0.054  # crank beam circular section diameter (updated from 0.048)
CB_NEL = 5       # crank beam Euler elements (updated from 3)
CONSTR_SPHERE = 0.012   # constraint visualization sphere size (updated from 0.01)
CRANK_VB_SPHERE = 0.014 # crank-vertical beam sphere size (updated from 0.01)
GLYPH_SCALE = 0.015     # glyph visualization scale (updated from 0.01)

# Derived positions
TRUSS_POS = chrono.ChVector3d(0, 0, 0)
CRANK_POS = chrono.ChVector3d(K / 2, H, 0)
VB_BASE = chrono.ChVector3d(0, 0, 0)
VB_TOP = chrono.ChVector3d(0, L, 0)

# === System setup ===
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Direct solver for stiff beam matrices
import pychrono.pardisomkl as mkl
sys.SetSolver(mkl.ChSolverPardisoMKL())

# HHT timestepper for stiff beam buckling
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# === FEA Mesh ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

# Material properties (aluminium)
density = 2700.0
E = 73e9          # Young's modulus
nu = 0.3          # Poisson ratio

# --- Vertical beam (cantilever, fixed at base) ---
sec_vertical = fea.ChBeamSectionEulerAdvanced()
sec_vertical.SetAsCircularSection(VB_DIAM)
sec_vertical.SetDensity(density)
sec_vertical.SetYoungModulus(E)
sec_vertical.SetShearModulusFromPoisson(nu)
sec_vertical.SetRayleighDamping(0.0)

builder_vb = fea.ChBuilderBeamEuler()
builder_vb.BuildBeam(
    mesh, sec_vertical, VB_NEL,
    VB_BASE, VB_TOP,
    chrono.VECT_Y,
)
# Fix root node (fixed cantilever)
builder_vb.GetLastBeamNodes().front().SetFixed(True)

# --- Horizontal beam (at height H, truss connected to vertical beam) ---
sec_horizontal = fea.ChBeamSectionEulerAdvanced()
sec_horizontal.SetAsRectangularSection(HB_WY, HB_WZ)
sec_horizontal.SetDensity(density)
sec_horizontal.SetYoungModulus(E)
sec_horizontal.SetShearModulusFromPoisson(nu)
sec_horizontal.SetRayleighDamping(0.0)

hb_start = chrono.ChVector3d(0, H, 0)
hb_end = chrono.ChVector3d(K, H, 0)

builder_hb = fea.ChBuilderBeamEuler()
builder_hb.BuildBeam(
    mesh, sec_horizontal, 3,
    hb_start, hb_end,
    chrono.VECT_Y,
)
# Connect horizontal beam to vertical beam at (0, H, 0)
# No additional constraint needed — shared node at that location

# --- Crank beam (from (K/2, H, 0) to crank center) ---
sec_crank = fea.ChBeamSectionEulerAdvanced()
sec_crank.SetAsCircularSection(CB_DIAM)
sec_crank.SetDensity(density)
sec_crank.SetYoungModulus(E)
sec_crank.SetShearModulusFromPoisson(nu)
sec_crank.SetRayleighDamping(0.0)

cb_start = chrono.ChVector3d(K / 2, H, 0)
cb_end = chrono.ChVector3d(K / 2, H - K / 2, 0)   # crank lower pivot

builder_cb = fea.ChBuilderBeamEuler()
builder_cb.BuildBeam(
    mesh, sec_crank, CB_NEL,
    cb_start, cb_end,
    chrono.VECT_Y,
)

sys.Add(mesh)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Buckling FEA — turn 2")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, 0.7, -1.2), chrono.ChVector3d(0.0, 0.5, 0.0))  # updated camera
vis.AddTypicalLights()

# FEA mesh visualization — two-shape pattern
vis_fea = chrono.ChVisualShapeFEA(mesh)
vis_fea.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_fea.SetColorscaleMinMax(-0.4, 0.4)
vis_fea.SetSmoothFaces(True)
vis_fea.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_fea)

vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(GLYPH_SCALE)  # updated glyph scale
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# === Structural body for crank mechanism ===
# Fixed support post at crank pivot location
crank_post = chrono.ChBody()
crank_post.SetMass(1.0)
crank_post.SetInertiaXX(chrono.ChVector3d(1e-6, 1e-6, 1e-6))
crank_post.SetPos(chrono.ChVector3d(K / 2, H, 0))
crank_post.SetFixed(True)
sys.AddBody(crank_post)

# Truss body (visual only — structural element)
truss_body = chrono.ChBody()
truss_body.SetMass(0.1)
truss_body.SetInertiaXX(chrono.ChVector3d(1e-6, 1e-6, 1e-6))
truss_body.SetPos(TRUSS_POS)
truss_body.SetFixed(True)
sys.AddBody(truss_body)

truss_vis = chrono.ChVisualShapeBox(W_SX, W_SY, W_SZ)
truss_vis.SetColor(chrono.ChColor(0.6, 0.6, 0.6))
truss_body.AddVisualShape(truss_vis, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))

# Crank body (visual element — the rotating arm)
crank_body = chrono.ChBody()
crank_body.SetMass(0.1)
crank_body.SetInertiaXX(chrono.ChVector3d(1e-6, 1e-6, 1e-6))
crank_body.SetPos(CRANK_POS)
sys.AddBody(crank_body)

crank_vis = chrono.ChVisualShapeBox(C_SX, C_SY, C_SZ)
crank_vis.SetColor(chrono.ChColor(0.5, 0.3, 0.3))
crank_body.AddVisualShape(crank_vis, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))

# Constraint sphere at crank-vertical connection
constraint_sphere = chrono.ChVisualShapeSphere(CONSTR_SPHERE)
constraint_sphere.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
crank_body.AddVisualShape(constraint_sphere, chrono.ChFramed(chrono.ChVector3d(K / 2, 0, 0), chrono.QUNIT))

# Constraint sphere between crank and vertical beam at crank pivot
crank_vb_sphere = chrono.ChVisualShapeSphere(CRANK_VB_SPHERE)
crank_vb_sphere.SetColor(chrono.ChColor(0.2, 0.8, 0.2))
crank_post.AddVisualShape(crank_vb_sphere, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))

# === Simulation loop ===
time_step = 1e-3
sim_end = 5.0

frame = 0
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(time_step)

# === Post-processing ===
