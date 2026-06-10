"""
Buckling demo — FEA multi-beam crank mechanism (PyChrono 9.0.0, Irrlicht).

System: ChSystemSMC + Pardiso MKL solver + HHT timestepper.
Components:
  - Fixed truss body (rigid, visualized as box)
  - Crank body (rigid, motor-driven around the truss)
  - Horizontal FEA beam (Euler-Bernoulli, rectangular section wy=0.12, wz=0.012)
  - Vertical FEA beam (Euler-Bernoulli, circular section d=0.03, 6 elements)
  - Crank FEA beam (Euler-Bernoulli, circular section d=0.054, 5 elements)
  - ChLinkMotorRotationSpeed driving the crank body relative to the truss
  - ChLinkMateGeneric constraints joining the FEA beams to the bodies/other beams
  - FEA visualization: bending moment Mz + node coordinate glyphs (glyph scale 0.015)
  - Camera at (0.0, 0.7, -1.2) looking at origin

Expected behavior: the crank rotates at constant speed, loading the connected beam
assembly; the vertical beam buckles under compressive load from the crank mechanism.

No contact needed: pure jointed FEA+rigid MBS, no collision shapes.
"""

# === Imports ===
import os
import math
import csv

import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr


# === Geometry and physics constants ===
L = 1.2          # beam length
H = 0.3          # height
K = 0.07         # crank length

# Truss visualization dimensions (full extents)
TRUSS_VIS = (0.03, 0.25, 0.12)

# Beam cross-section parameters
HZ_BEAM_WY  = 0.12    # horizontal beam width Y
HZ_BEAM_WZ  = 0.012   # horizontal beam width Z
VERT_BEAM_D = 0.03    # vertical beam diameter
VERT_N_ELEM = 6       # vertical beam number of Euler elements
CRANK_BEAM_D = 0.054  # crank beam diameter
CRANK_N_ELEM = 5      # crank beam number of Euler elements

# Constraint visualization sphere radii
CONSTR_SPHERE_R      = 0.012  # general constraint sphere
CRANK_VERT_SPHERE_R  = 0.014  # crank-to-vertical beam sphere

# Glyph scale
GLYPH_SCALE = 0.015

# Camera
CAM_POS = chrono.ChVector3d(0.0, 0.7, -1.2)
CAM_TGT = chrono.ChVector3d(0.0, 0.0, 0.0)

# Simulation timing
TIME_STEP = 1e-3       # HHT timestep for stiff Euler beams
SIM_END   = 5.0        # simulation duration (seconds)
RENDER_FPS = 50.0      # render frame rate
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Crank motor speed (rad/s)
CRANK_OMEGA = 1.0

# === System + solver + timestepper ===
# FEA beam scenes use ChSystemSMC + Pardiso MKL + HHT
# Pure jointed FEA — no collision shapes, no SetCollisionSystemType needed.
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # Y-up convention for FEA

solver = mkl.ChSolverPardisoMKL()
sys.SetSolver(solver)

sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)  # stable linearized implicit for stiff beams

# === Truss body (fixed reference for the mechanism) ===
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetPos(chrono.ChVector3d(0, 0, 0))
# Visual shape for the truss
truss_shape = chrono.ChVisualShapeBox(TRUSS_VIS[0], TRUSS_VIS[1], TRUSS_VIS[2])
truss.AddVisualShape(truss_shape)
sys.Add(truss)

# === Crank body ===
crank = chrono.ChBody()
crank.SetPos(chrono.ChVector3d(0, 0, 0))
crank.SetMass(0.1)
crank.SetInertiaXX(chrono.ChVector3d(0.001, 0.001, 0.001))
# Visual shape for the crank
crank_shape = chrono.ChVisualShapeBox(K, 0.03, 0.03)
crank.AddVisualShape(crank_shape,
                     chrono.ChFramed(chrono.ChVector3d(K / 2, 0, 0), chrono.QUNIT))
sys.Add(crank)

# === Motor: truss → crank (rotates around Z axis) ===
motor = chrono.ChLinkMotorRotationSpeed()
motor_frame = chrono.ChFramed(chrono.ChVector3d(0, 0, 0),
                              chrono.Q_ROTATE_Y_TO_Z)  # rotation axis = Z
motor.Initialize(crank, truss, motor_frame)
motor_fun = chrono.ChFunctionConst(CRANK_OMEGA)
motor.SetSpeedFunction(motor_fun)
sys.Add(motor)

# === FEA mesh ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(False)  # gravity handled at system level; beams are loaded by constraints

# FEA beam: no contact material needed — driven by constraints and motor only.

# --- Horizontal beam (rectangular section) ---
sec_hz = fea.ChBeamSectionEulerAdvanced()
sec_hz.SetAsRectangularSection(HZ_BEAM_WY, HZ_BEAM_WZ)
sec_hz.SetDensity(2700.0)     # aluminum density
sec_hz.SetYoungModulus(73e9)
sec_hz.SetShearModulusFromPoisson(0.3)
sec_hz.SetRayleighDamping(0.000)

builder_hz = fea.ChBuilderBeamEuler()
builder_hz.BuildBeam(
    mesh, sec_hz, 3,
    chrono.ChVector3d(0.0, 0.0, 0.0),
    chrono.ChVector3d(L,   0.0, 0.0),
    chrono.ChVector3d(0, 1, 0),  # Y as beam up direction
)
hz_nodes_ref = builder_hz.GetLastBeamNodes()  # cache: store ref before indexing
hz_nodes = [hz_nodes_ref[i] for i in range(hz_nodes_ref.size())]

# Fix the start node of the horizontal beam to the truss
constr_hz_start = chrono.ChLinkMateGeneric()
constr_hz_start.Initialize(hz_nodes[0], truss,
                            False,
                            hz_nodes[0].Frame(),
                            hz_nodes[0].Frame())
constr_hz_start.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(constr_hz_start)

# Constraint visualization sphere at start of horizontal beam
hz_start_shape = chrono.ChVisualShapeSphere(CONSTR_SPHERE_R)
hz_start_shape.SetColor(chrono.ChColor(0.7, 0.3, 0.3))
truss.AddVisualShape(hz_start_shape,
                     chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))

# --- Vertical beam (circular section, 6 elements) ---
sec_vert = fea.ChBeamSectionEulerAdvanced()
sec_vert.SetAsCircularSection(VERT_BEAM_D)
sec_vert.SetDensity(2700.0)
sec_vert.SetYoungModulus(73e9)
sec_vert.SetShearModulusFromPoisson(0.3)
sec_vert.SetRayleighDamping(0.000)

# Vertical beam runs along Y from the end of horizontal beam
vert_start = chrono.ChVector3d(L, 0.0, 0.0)
vert_end   = chrono.ChVector3d(L, H,   0.0)

builder_vert = fea.ChBuilderBeamEuler()
builder_vert.BuildBeam(
    mesh, sec_vert, VERT_N_ELEM,
    vert_start,
    vert_end,
    chrono.ChVector3d(1, 0, 0),  # X as up direction for vertical beam
)
vert_nodes_ref = builder_vert.GetLastBeamNodes()  # cache
vert_nodes = [vert_nodes_ref[i] for i in range(vert_nodes_ref.size())]

# Connect bottom of vertical beam to end of horizontal beam (continuity constraint)
constr_vert_hz = chrono.ChLinkMateGeneric()
constr_vert_hz.Initialize(hz_nodes[-1], vert_nodes[0],
                           False,
                           hz_nodes[-1].Frame(),
                           vert_nodes[0].Frame())
constr_vert_hz.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(constr_vert_hz)

# Visualization sphere at the junction of horizontal and vertical beams
hz_end_shape = chrono.ChVisualShapeSphere(CONSTR_SPHERE_R)
hz_end_shape.SetColor(chrono.ChColor(0.3, 0.7, 0.3))
truss.AddVisualShape(hz_end_shape,
                     chrono.ChFramed(vert_start, chrono.QUNIT))

# --- Crank beam (circular section, 5 elements) ---
sec_crank_beam = fea.ChBeamSectionEulerAdvanced()
sec_crank_beam.SetAsCircularSection(CRANK_BEAM_D)
sec_crank_beam.SetDensity(2700.0)
sec_crank_beam.SetYoungModulus(73e9)
sec_crank_beam.SetShearModulusFromPoisson(0.3)
sec_crank_beam.SetRayleighDamping(0.000)

# Crank beam from crank tip to top of vertical beam
crank_tip  = chrono.ChVector3d(K, 0.0, 0.0)
vert_top   = vert_end

builder_crank_beam = fea.ChBuilderBeamEuler()
builder_crank_beam.BuildBeam(
    mesh, sec_crank_beam, CRANK_N_ELEM,
    crank_tip,
    vert_top,
    chrono.ChVector3d(0, 1, 0),
)
cb_nodes_ref = builder_crank_beam.GetLastBeamNodes()  # cache
cb_nodes = [cb_nodes_ref[i] for i in range(cb_nodes_ref.size())]

# Connect start of crank beam to the crank body
constr_crank_start = chrono.ChLinkMateGeneric()
constr_crank_start.Initialize(cb_nodes[0], crank,
                               False,
                               cb_nodes[0].Frame(),
                               cb_nodes[0].Frame())
constr_crank_start.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(constr_crank_start)

# Connect end of crank beam to top of vertical beam (crank–vertical beam junction)
constr_crank_vert = chrono.ChLinkMateGeneric()
constr_crank_vert.Initialize(cb_nodes[-1], vert_nodes[-1],
                              False,
                              cb_nodes[-1].Frame(),
                              vert_nodes[-1].Frame())
constr_crank_vert.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(constr_crank_vert)

# Visualization sphere at crank-to-vertical-beam junction
cv_sphere = chrono.ChVisualShapeSphere(CRANK_VERT_SPHERE_R)
cv_sphere.SetColor(chrono.ChColor(0.3, 0.3, 0.8))
truss.AddVisualShape(cv_sphere,
                     chrono.ChFramed(vert_top, chrono.QUNIT))

# Register the mesh with the system
sys.Add(mesh)

# === FEA Visualization ===
# Shape 1 — bending moment Mz field on beam surface
vis_surface = chrono.ChVisualShapeFEA()
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surface.SetColormapRange(-0.4, 0.4)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

# Shape 2 — node coordinate system glyphs
vis_glyph = chrono.ChVisualShapeFEA()
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(GLYPH_SCALE)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# === Visualization (Irrlicht) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Buckling FEA Demo — Crank Mechanism (Turn 2)")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)  # Y-up for this FEA scene
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(CAM_POS, CAM_TGT)
vis.AddTypicalLights()
vis.AddGrid(
    0.1, 0.1, 20, 20,
    chrono.ChCoordsysd(chrono.ChVector3d(0, -0.01, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

# === Review-only recording setup ===

# === Main loop ===
frame = 0

try:

    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            t = sys.GetChTime()


            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:  # solver divergence or bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
