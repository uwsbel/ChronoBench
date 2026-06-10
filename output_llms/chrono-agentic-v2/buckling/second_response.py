"""
Buckling FEA Simulation — Turn 2 parameter update.

Models a structural buckling scenario using PyChrono FEA with:
  - ChSystemSMC + Pardiso MKL solver + HHT timestepper
  - A fixed truss body and a crank body driven by a motor
  - Three FEA beam segments: horizontal beam (rectangular section),
    vertical beam (circular section, Euler), and crank beam (circular section, Euler)
  - Constraints connecting the beams to truss and crank bodies
  - Irrlicht visualization with FEA surface and glyph shapes

Updated parameters (Turn 2):
  L=1.2, H=0.3, K=0.07; horizontal beam wy=0.12/wz=0.012;
  vertical beam dia=0.03 with 6 Euler elements;
  crank beam dia=0.054 with 5 Euler elements;
  truss vis box=(0.03,0.25,0.12); crank vis box=(K,0.03,0.03);
  constraint sphere size=0.012 (truss end) / 0.014 (crank-vertical junction);
  glyph scale=0.015; camera at (0.0, 0.7, -1.2).

System: ChSystemSMC (required for FEA beam/buckling demos).
No contact surfaces needed — driven by constraints + motor only.
"""

import os
import math
import csv

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

# === Named constants ===
# Geometry parameters (Turn 2 updated values)
L = 1.2          # beam length
H = 0.3          # height parameter
K = 0.07         # crank length

# Beam cross-section parameters
HORIZ_WY = 0.12    # horizontal beam width in Y
HORIZ_WZ = 0.012   # horizontal beam width in Z
VERT_DIA  = 0.03   # vertical beam circular diameter
VERT_NEL  = 6      # vertical beam number of Euler elements
CRANK_DIA = 0.054  # crank beam circular diameter
CRANK_NEL = 5      # crank beam number of Euler elements

# Visualization sizes
TRUSS_VIS  = (0.03, 0.25, 0.12)   # truss visualization box
CRANK_VIS  = (K, 0.03, 0.03)      # crank visualization box
CONSTR_SZ1 = 0.012                 # constraint sphere size (truss end)
CONSTR_SZ2 = 0.014                 # constraint sphere size (crank-vertical junction)
GLYPH_SCALE = 0.015                # glyph symbols scale

# Simulation parameters
time_step  = 0.001
sim_end    = 6.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# Material properties (aluminium-like)
DENSITY = 2700.0
E_MOD   = 73e9
POISSON = 0.3
RAYLEIGH_DAMP = 0.000

# Motor speed
MOTOR_SPEED = 0.5  # rad/s

# === System & gravity ===
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # Y-up convention

# Pardiso MKL solver — required for stiff Euler beam elements
sys.SetSolver(mkl.ChSolverPardisoMKL())

# HHT timestepper (canonical-minimal, matches buckling truth)
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)
# FEA beam: no contact material needed — driven by constraints + gravity + motor only

# === Bodies ===
# Truss (fixed reference body)
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetPos(chrono.ChVector3d(0, 0, 0))
vis_truss = chrono.ChVisualShapeBox(TRUSS_VIS[0], TRUSS_VIS[1], TRUSS_VIS[2])
vis_truss.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
truss.AddVisualShape(vis_truss)
sys.Add(truss)

# Crank body (driven by motor about truss)
crank = chrono.ChBody()
crank.SetPos(chrono.ChVector3d(0, -H, 0))
vis_crank = chrono.ChVisualShapeBox(CRANK_VIS[0], CRANK_VIS[1], CRANK_VIS[2])
vis_crank.SetColor(chrono.ChColor(0.6, 0.4, 0.2))
crank.AddVisualShape(vis_crank)
sys.Add(crank)

# Motor: rotates crank about the Z-axis relative to truss
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(
    crank,
    truss,
    chrono.ChFramed(chrono.ChVector3d(0, -H, 0), chrono.QUNIT)
)
motor_speed_func = chrono.ChFunctionConst(MOTOR_SPEED)
motor.SetSpeedFunction(motor_speed_func)
sys.Add(motor)

# === FEA Mesh ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

# --- Horizontal Beam (rectangular cross-section) ---
sec_h = fea.ChBeamSectionEulerAdvanced()
sec_h.SetAsRectangularSection(HORIZ_WY, HORIZ_WZ)
sec_h.SetDensity(DENSITY)
sec_h.SetYoungModulus(E_MOD)
sec_h.SetShearModulusFromPoisson(POISSON)
sec_h.SetRayleighDamping(RAYLEIGH_DAMP)

builder_h = fea.ChBuilderBeamEuler()
builder_h.BuildBeam(
    mesh, sec_h, 3,
    chrono.ChVector3d(0, 0, 0),       # start at truss origin
    chrono.ChVector3d(L, 0, 0),       # along X-axis
    chrono.ChVector3d(0, 1, 0),       # up direction
)
beam_h_nodes = builder_h.GetLastBeamNodes()  # cache: SWIG GC guard + reused below
nodes_h = [beam_h_nodes[i] for i in range(beam_h_nodes.size())]

# --- Vertical Beam (circular cross-section, 6 Euler elements) ---
sec_v = fea.ChBeamSectionEulerAdvanced()
sec_v.SetAsCircularSection(VERT_DIA)
sec_v.SetDensity(DENSITY)
sec_v.SetYoungModulus(E_MOD)
sec_v.SetShearModulusFromPoisson(POISSON)
sec_v.SetRayleighDamping(RAYLEIGH_DAMP)

builder_v = fea.ChBuilderBeamEuler()
builder_v.BuildBeam(
    mesh, sec_v, VERT_NEL,
    chrono.ChVector3d(L, 0, 0),          # starts at horizontal beam tip
    chrono.ChVector3d(L, -H, 0),         # goes downward
    chrono.ChVector3d(1, 0, 0),          # lateral reference
)
beam_v_nodes = builder_v.GetLastBeamNodes()  # cache: SWIG GC guard + reused below
nodes_v = [beam_v_nodes[i] for i in range(beam_v_nodes.size())]

# --- Crank Beam (circular cross-section, 5 Euler elements) ---
sec_c = fea.ChBeamSectionEulerAdvanced()
sec_c.SetAsCircularSection(CRANK_DIA)
sec_c.SetDensity(DENSITY)
sec_c.SetYoungModulus(E_MOD)
sec_c.SetShearModulusFromPoisson(POISSON)
sec_c.SetRayleighDamping(RAYLEIGH_DAMP)

builder_c = fea.ChBuilderBeamEuler()
builder_c.BuildBeam(
    mesh, sec_c, CRANK_NEL,
    chrono.ChVector3d(0, -H, 0),          # at crank pivot
    chrono.ChVector3d(K, -H, 0),          # extends along X
    chrono.ChVector3d(0, 1, 0),           # up reference
)
beam_c_nodes = builder_c.GetLastBeamNodes()  # cache: SWIG GC guard + reused below
nodes_c = [beam_c_nodes[i] for i in range(beam_c_nodes.size())]

sys.Add(mesh)

# === Joints / constraints ===
# Constraint 1: fix horizontal beam root node to truss (all 6 DOF)
constr_root = chrono.ChLinkMateGeneric()
constr_root.Initialize(
    nodes_h[0], truss, False,
    nodes_h[0].Frame(), nodes_h[0].Frame()
)
constr_root.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(constr_root)

# Constraint 2: fix horizontal beam tip to vertical beam top (all 6 DOF)
constr_hv = chrono.ChLinkMateGeneric()
constr_hv.Initialize(
    nodes_h[-1], nodes_v[0], False,
    nodes_h[-1].Frame(), nodes_v[0].Frame()
)
constr_hv.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(constr_hv)

# Constraint 3: fix crank beam root to crank body
constr_crank_root = chrono.ChLinkMateFix()
constr_crank_root.Initialize(nodes_c[0], crank)
sys.Add(constr_crank_root)

# Constraint 4: connect crank beam tip to vertical beam bottom
constr_cv = chrono.ChLinkMateGeneric()
constr_cv.Initialize(
    nodes_c[-1], nodes_v[-1], False,
    nodes_c[-1].Frame(), nodes_v[-1].Frame()
)
constr_cv.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(constr_cv)

# Constraint sphere visual markers at key junction points
# Sphere at truss-horizontal beam attachment (size CONSTR_SZ1)
sp1 = chrono.ChBody(); sp1.SetFixed(True)
sp1.SetPos(nodes_h[0].GetPos())
vis_sp1 = chrono.ChVisualShapeSphere(CONSTR_SZ1)
vis_sp1.SetColor(chrono.ChColor(0.9, 0.1, 0.1))
sp1.AddVisualShape(vis_sp1)
sys.Add(sp1)

# Sphere at crank beam - vertical beam junction (size CONSTR_SZ2)
sp2 = chrono.ChBody(); sp2.SetFixed(True)
sp2.SetPos(nodes_v[-1].GetPos())
vis_sp2 = chrono.ChVisualShapeSphere(CONSTR_SZ2)
vis_sp2.SetColor(chrono.ChColor(0.1, 0.9, 0.1))
sp2.AddVisualShape(vis_sp2)
sys.Add(sp2)

# === FEA Visualization shapes (attach before Initialize) ===
# Shape 1: surface/scalar showing bending moment Mz
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surface.SetColorscaleMinMax(-0.4, 0.4)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

# Shape 2: glyph node markers
vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(GLYPH_SCALE)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Buckling FEA Simulation")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)  # Y-up convention
vis.Initialize()                                   # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(0.0, 0.7, -1.2),   # camera position (Turn 2)
    chrono.ChVector3d(0.5, -0.1, 0.0)    # look-at target
)
vis.AddTypicalLights()

# === Review-only setup ===

frame = 0

# === Main loop ===
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad beam state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
