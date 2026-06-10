"""
Beam Buckling FEA Simulation — PyChrono 9.0.x / Irrlicht

Models a beam buckling scenario with:
  - ChSystemSMC + Pardiso MKL direct solver + HHT timestepper (required for stiff FEA beams)
  - Two Euler-Bernoulli FEA beam columns (Beam A and Beam B) built from ChBuilderBeamEuler
  - A rigid platen body driven by a custom motor function (CustomDisplacementFunc subclass
    of ChFunction) using ChLinkMotorLinearPosition — which is itself a full linear-motor-link
    (prescribes position AND provides the prismatic constraint; no separate prismatic needed)
  - Constraints between beam nodes and rigid bodies using ChLinkMateGeneric (correct for
    ChNodeFEAxyzrot nodes produced by ChBuilderBeamEuler):
      * Column bases clamped all-6-DOF to ground (fixed base boundary condition)
      * Column tips pinned in translation (tx, ty, tz) to the moving platen, free rotation
  - FEA visualization: one ChVisualShapeFEA surface shape (bending moment Mz) + one
    glyph shape (node coordinate systems) per mesh
  - Irrlicht real-time window for 3D rendering
  - Y-up world convention (FEA truth); gravity disabled on FEA meshes (motor drives loading)

Expected behavior: both flexible beam columns develop lateral deflections as the
motor-driven platen descends and applies compressive axial loads, exhibiting classic
Euler buckling lateral instability after the critical load is exceeded.
"""

# === Imports ===
import os
import math
import csv
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# === Simulation parameters ===
time_step    = 1e-3          # s — stiff beam timestep
sim_end      = 5.0           # s — time to see buckling develop
render_fps   = 50.0          # Hz — review video frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# === Geometry / physics constants ===
BEAM_L    = 1.0       # m — beam column height
BEAM_D    = 0.012     # m — circular cross-section diameter (thin -> lower critical load)
BEAM_E    = 73e9      # Pa — Young's modulus (aluminium)
BEAM_RHO  = 2700.0    # kg/m³ — density
BEAM_NU   = 0.3       # Poisson ratio
N_ELEM    = 12        # elements per beam column
BEAM_SEP  = 0.15      # m — lateral separation between two beam columns

# === Custom motor function: prescribed axial compression (ChFunction subclass) ===
class CustomDisplacementFunc(chrono.ChFunction):
    """Prescribes platen downward displacement: linear ramp then constant hold.
    Returns displacement (m) — used with ChLinkMotorLinearPosition.
    The motor's local-Z axis points down (-Y in world) due to frame orientation,
    so positive return value moves the platen downward (compressing the beams).
    """
    def __init__(self, ramp_end=2.5, max_disp=0.12):
        chrono.ChFunction.__init__(self)   # MUST call base ctor
        self._ramp_end = ramp_end
        self._max_disp = max_disp

    def GetVal(self, x):                    # x = time (s); return motor position (m)
        if x <= 0.0:
            return 0.0
        if x >= self._ramp_end:
            return self._max_disp
        return self._max_disp * (x / self._ramp_end)


# === System & solver setup ===
# FEA beam scenes require ChSystemSMC + Pardiso MKL + HHT timestepper.
# Pure jointed FEA (beam-node constraints + motor), no rigid-body collision shapes.
# SetCollisionSystemType NOT needed: no contact/collision in this scene.
# FEA beam: no contact material needed — driven by constraints + motor only.

sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # Y-up world

# Pardiso MKL direct solver — required for stiff Euler beam stiffness matrices
sys.SetSolver(mkl.ChSolverPardisoMKL())

# HHT timestepper — canonical minimal form (exactly as in FEA truth scripts)
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# === Ground truss (fixed reference body) ===
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetName("ground")
sys.Add(ground)

# Visual: flat floor reference box
floor_shape = chrono.ChVisualShapeBox(0.5, 0.01, 0.3)
floor_shape.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
ground.AddVisualShape(floor_shape,
    chrono.ChFramed(chrono.ChVector3d(0, -0.005, 0), chrono.QUNIT))

# === Rigid platen body (motor-driven, compresses beam column tops) ===
platen = chrono.ChBody()
platen.SetName("platen")
platen.SetMass(0.5)
platen.SetInertiaXX(chrono.ChVector3d(0.001, 0.001, 0.001))
platen.SetPos(chrono.ChVector3d(0, BEAM_L, 0))   # initial: top of beams
sys.Add(platen)

platen_shape = chrono.ChVisualShapeBox(0.35, 0.015, 0.2)
platen_shape.SetColor(chrono.ChColor(0.8, 0.3, 0.1))
platen.AddVisualShape(platen_shape, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))

# === Motor: ChLinkMotorLinearPosition (full linear motor-link, no extra prismatic needed) ===
# ChLinkMotorLinearPosition prescribes position AND acts as the prismatic constraint
# between platen and ground. Local-Z of the frame is the motor axis.
# Orient frame so local-Z = world -Y (downward) using 90° rotation about world +X:
# q_down: local Z → world -Y means rotate 90° about world X (positive)
q_motor = chrono.QuatFromAngleAxis(chrono.CH_PI_2, chrono.VECT_X)
motor_func = CustomDisplacementFunc(ramp_end=2.5, max_disp=0.12)
platen_motor = chrono.ChLinkMotorLinearPosition()
platen_motor.Initialize(platen, ground,
    chrono.ChFramed(chrono.ChVector3d(0, BEAM_L, 0), q_motor))
platen_motor.SetMotorFunction(motor_func)
sys.AddLink(platen_motor)

# Lateral guide: ChLinkLockSpherical or planar link to prevent platen from rotating freely
# while the motor constrains axial position. Use a planar constraint on ground.
# ChLinkLockPlanar: constrains the platen to move in the XZ plane (leaves Y free for motor).
# Wait — ChLinkMotorLinearPosition already constrains 5 DOF (fixes lateral + rotations).
# It is a FULL motor-link similar to a prismatic + position servo.
# No additional lateral guide needed.

# === FEA Mesh A: left beam column ===
# Column at X = -BEAM_SEP/2, runs along Y from 0 to BEAM_L.
# SetAutomaticGravity(False): motor provides the axial load.
mesh_a = fea.ChMesh()
mesh_a.SetAutomaticGravity(False)

sec_a = fea.ChBeamSectionEulerAdvanced()
sec_a.SetAsCircularSection(BEAM_D)
sec_a.SetDensity(BEAM_RHO)
sec_a.SetYoungModulus(BEAM_E)
sec_a.SetShearModulusFromPoisson(BEAM_NU)
sec_a.SetRayleighDamping(0.002)

builder_a = fea.ChBuilderBeamEuler()
builder_a.BuildBeam(
    mesh_a, sec_a, N_ELEM,
    chrono.ChVector3d(-BEAM_SEP / 2, 0.0, 0.0),      # base
    chrono.ChVector3d(-BEAM_SEP / 2, BEAM_L, 0.0),   # tip
    chrono.ChVector3d(1, 0, 0),                        # lateral reference direction
)
sys.Add(mesh_a)

# SWIG GC pitfall: store container reference before indexing to prevent dangling ptrs
beam_nodes_a = builder_a.GetLastBeamNodes()
nodes_a = [beam_nodes_a[i] for i in range(beam_nodes_a.size())]  # cache: keep alive

node_a_base = nodes_a[0]   # bottom node
node_a_tip  = nodes_a[-1]  # top node

# === FEA Mesh B: right beam column ===
mesh_b = fea.ChMesh()
mesh_b.SetAutomaticGravity(False)

sec_b = fea.ChBeamSectionEulerAdvanced()
sec_b.SetAsCircularSection(BEAM_D)
sec_b.SetDensity(BEAM_RHO)
sec_b.SetYoungModulus(BEAM_E)
sec_b.SetShearModulusFromPoisson(BEAM_NU)
sec_b.SetRayleighDamping(0.002)

builder_b = fea.ChBuilderBeamEuler()
builder_b.BuildBeam(
    mesh_b, sec_b, N_ELEM,
    chrono.ChVector3d(+BEAM_SEP / 2, 0.0, 0.0),
    chrono.ChVector3d(+BEAM_SEP / 2, BEAM_L, 0.0),
    chrono.ChVector3d(1, 0, 0),
)
sys.Add(mesh_b)

beam_nodes_b = builder_b.GetLastBeamNodes()
nodes_b = [beam_nodes_b[i] for i in range(beam_nodes_b.size())]

node_b_base = nodes_b[0]
node_b_tip  = nodes_b[-1]

# === Constraints: clamp column bases to ground (all 6 DOF — fixed base BCs) ===
# ChBuilderBeamEuler creates ChNodeFEAxyzrot nodes → use ChLinkMateGeneric.
constr_a_base = chrono.ChLinkMateGeneric()
constr_a_base.Initialize(node_a_base, ground, False,
                          node_a_base.Frame(), node_a_base.Frame())
constr_a_base.SetConstrainedCoords(True, True, True, True, True, True)
constr_a_base.SetName("constr_a_base")
sys.Add(constr_a_base)

constr_b_base = chrono.ChLinkMateGeneric()
constr_b_base.Initialize(node_b_base, ground, False,
                          node_b_base.Frame(), node_b_base.Frame())
constr_b_base.SetConstrainedCoords(True, True, True, True, True, True)
constr_b_base.SetName("constr_b_base")
sys.Add(constr_b_base)

# === Constraints: pin column tips to platen (translation only — pin-ended top BC) ===
# Constrain tx, ty, tz; free rx, ry, rz (allow rotation at beam top).
constr_a_tip = chrono.ChLinkMateGeneric()
constr_a_tip.Initialize(node_a_tip, platen, False,
                         node_a_tip.Frame(), node_a_tip.Frame())
constr_a_tip.SetConstrainedCoords(True, True, True, False, False, False)
constr_a_tip.SetName("constr_a_tip")
sys.Add(constr_a_tip)

constr_b_tip = chrono.ChLinkMateGeneric()
constr_b_tip.Initialize(node_b_tip, platen, False,
                         node_b_tip.Frame(), node_b_tip.Frame())
constr_b_tip.SetConstrainedCoords(True, True, True, False, False, False)
constr_b_tip.SetName("constr_b_tip")
sys.Add(constr_b_tip)

# === FEA Visualization: two ChVisualShapeFEA shapes per mesh ===
# Must be attached to mesh BEFORE vis.Initialize() so Irrlicht device picks them up.

# Beam A — shape 1: surface colored by bending moment Mz
vis_surf_a = chrono.ChVisualShapeFEA(mesh_a)
vis_surf_a.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surf_a.SetColorscaleMinMax(-0.4, 0.4)
vis_surf_a.SetSmoothFaces(True)
vis_surf_a.SetWireframe(False)
mesh_a.AddVisualShapeFEA(vis_surf_a)

# Beam A — shape 2: glyph (node coordinate system triads)
vis_glyph_a = chrono.ChVisualShapeFEA(mesh_a)
vis_glyph_a.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph_a.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph_a.SetSymbolsThickness(0.006)
vis_glyph_a.SetSymbolsScale(0.01)
vis_glyph_a.SetZbufferHide(False)
mesh_a.AddVisualShapeFEA(vis_glyph_a)

# Beam B — shape 1: surface colored by bending moment Mz
vis_surf_b = chrono.ChVisualShapeFEA(mesh_b)
vis_surf_b.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surf_b.SetColorscaleMinMax(-0.4, 0.4)
vis_surf_b.SetSmoothFaces(True)
vis_surf_b.SetWireframe(False)
mesh_b.AddVisualShapeFEA(vis_surf_b)

# Beam B — shape 2: glyph (node coordinate system triads)
vis_glyph_b = chrono.ChVisualShapeFEA(mesh_b)
vis_glyph_b.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph_b.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph_b.SetSymbolsThickness(0.006)
vis_glyph_b.SetSymbolsScale(0.01)
vis_glyph_b.SetZbufferHide(False)
mesh_b.AddVisualShapeFEA(vis_glyph_b)

# === Visualization (Irrlicht window) ===
# Irrlicht call order: Initialize() FIRST, then scene elements AFTER.
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Beam Buckling FEA — PyChrono 9.0.x")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # Y-up for FEA scene
vis.Initialize()                                     # FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.5, 0.8, 1.4), chrono.ChVector3d(0, 0.5, 0))
vis.AddTypicalLights()
vis.AddGrid(0.1, 0.1, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === CSV logging setup ===

# === Pre-loop caches ===
mid_idx    = N_ELEM // 2
node_a_mid = nodes_a[mid_idx]   # cache: mid-beam node A for lateral deflection
node_b_mid = nodes_b[mid_idx]   # cache: mid-beam node B for lateral deflection

# === Main simulation loop ===
frame = 0

try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t = sys.GetChTime()
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad FEA state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing (review-only) ===
