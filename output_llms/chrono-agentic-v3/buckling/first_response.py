"""
Beam buckling simulation using PyChrono FEA (Euler-Bernoulli elements).

Models a vertical column undergoing buckling driven by a custom motor function.
The system consists of:
  - A vertical FEA column beam (fixed at base) — the primary buckling member
  - A horizontal FEA crank arm (connected to the column top) — load applicator
  - A custom ChFunction subclass prescribing a sinusoidal rotation angle at the
    crank tip via a ChLinkMotorRotationAngle, which imposes lateral displacement
    on the column top through the ChLinkMateGeneric coupling
  - ChLinkMateGeneric constraints linking column top node to crank start node,
    and crank tip to truss, implementing rigid connections between different parts

System type: ChSystemSMC
Solver: Pardiso MKL (direct, required for stiff Euler beams)
Timestepper: Euler Implicit Linearized (stable for forced FEA response)
World: Y-up (gravity = (0, -9.81, 0))
FEA beam: no contact material needed — driven by constraints + gravity + motor only
Expected behavior: column top deflects laterally following the motor angle profile;
  beam moment field shows bending at the column base indicating buckling loads.
"""

import math
import os
import csv

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr


# === Named constants ===
SIM_END = 10.0          # simulation duration (s)
TIME_STEP = 1e-3        # timestep for stiff Euler beams
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Column beam geometry
COL_LENGTH = 1.2        # m — vertical column length
COL_DIAM = 0.018        # m — circular cross-section diameter
COL_DENSITY = 2700.0    # kg/m³ (aluminium)
COL_E = 73e9            # Pa — Young's modulus

# Crank arm beam geometry (horizontal arm, load applicator)
ARM_LENGTH = 0.3        # m
ARM_WIDTH = 0.020       # m — rectangular cross-section wy
ARM_HEIGHT = 0.020      # m — rectangular cross-section wz

# Motor parameters
MOTOR_AMP = 0.18        # rad — peak crank angle
MOTOR_PERIOD = 3.0      # s — oscillation period


# === Custom motor function — prescribes crank rotation to impose buckling load ===
class BucklingAngle(chrono.ChFunction):
    """Prescribes a slow sinusoidal crank rotation that imposes lateral load on the column."""

    def __init__(self):
        chrono.ChFunction.__init__(self)  # MUST call base ctor

    def GetVal(self, t):
        """Return crank angle (rad) at time t."""
        return MOTOR_AMP * math.sin(math.pi * t / MOTOR_PERIOD)

    def Update(self, x):
        """Called by SWIG director — no-op for a stateless function."""
        pass

    def Clone(self):
        return BucklingAngle()


# === System & gravity ===
# FEA buckling: ChSystemSMC + Pardiso MKL solver + Euler Implicit Linearized (Y-up)
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
# Pure jointed FEA (no rigid-body contact) — SetCollisionSystemType omitted per FEA skill

# === Solver (Pardiso MKL — direct, required for stiff beam stiffness matrices) ===
sys.SetSolver(mkl.ChSolverPardisoMKL())

# === Timestepper (Euler Implicit Linearized — stable for forced FEA response) ===
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)

# === FEA mesh 1 — vertical buckling column ===
mesh_col = fea.ChMesh()
mesh_col.SetAutomaticGravity(True)

sec_col = fea.ChBeamSectionEulerAdvanced()
sec_col.SetAsCircularSection(COL_DIAM)
sec_col.SetDensity(COL_DENSITY)
sec_col.SetYoungModulus(COL_E)
sec_col.SetShearModulusFromPoisson(0.3)
sec_col.SetRayleighDamping(0.001)

up_col = chrono.ChVector3d(1, 0, 0)  # lateral reference direction for beam section
builder_col = fea.ChBuilderBeamEuler()
builder_col.BuildBeam(
    mesh_col, sec_col, 16,
    chrono.ChVector3d(0, 0, 0),           # base (fixed end)
    chrono.ChVector3d(0, COL_LENGTH, 0),  # top (free end, Y-up)
    up_col,
)

# Store beam nodes — SWIG GC: must keep container reference before indexing
col_nodes_container = builder_col.GetLastBeamNodes()
col_nodes = [col_nodes_container[i] for i in range(col_nodes_container.size())]

# Fix base of column to ground (pinned/clamped boundary condition)
col_nodes[0].SetFixed(True)

sys.Add(mesh_col)

# === FEA mesh 2 — horizontal crank arm (load applicator) ===
# Rectangular section; gravity disabled (arm is motor-driven, not gravity-loaded)
mesh_arm = fea.ChMesh()
mesh_arm.SetAutomaticGravity(False)

sec_arm = fea.ChBeamSectionEulerAdvanced()
sec_arm.SetAsRectangularSection(ARM_WIDTH, ARM_HEIGHT)
sec_arm.SetDensity(COL_DENSITY)
sec_arm.SetYoungModulus(COL_E)
sec_arm.SetShearModulusFromPoisson(0.3)
sec_arm.SetRayleighDamping(0.001)

arm_start = chrono.ChVector3d(0, COL_LENGTH, 0)       # at column top
arm_end = chrono.ChVector3d(ARM_LENGTH, COL_LENGTH, 0)  # extends horizontally
up_arm = chrono.ChVector3d(0, 1, 0)  # section reference direction for arm

builder_arm = fea.ChBuilderBeamEuler()
builder_arm.BuildBeam(
    mesh_arm, sec_arm, 5,
    arm_start,
    arm_end,
    up_arm,
)

# Store arm nodes (SWIG GC prevention — keep container reference)
arm_nodes_container = builder_arm.GetLastBeamNodes()
arm_nodes = [arm_nodes_container[i] for i in range(arm_nodes_container.size())]

sys.Add(mesh_arm)

# === Constraint 1 — column top node to arm start node (rigid coupling) ===
# ChLinkMateGeneric: all 6 DOF constrained; this connects two different FEA parts
link_col_arm = chrono.ChLinkMateGeneric()
link_col_arm.Initialize(
    col_nodes[-1],    # column top node (ChNodeFEAxyzrot)
    arm_nodes[0],     # arm start node  (ChNodeFEAxyzrot)
    False,            # frames are in world coords (not body-local)
    col_nodes[-1].Frame(),
    arm_nodes[0].Frame(),
)
link_col_arm.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(link_col_arm)

# === Fixed truss (ground body for motor anchor) ===
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetMass(1.0)
truss.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
sys.Add(truss)

# === Motor body — lightweight rigid body at arm tip for motor attachment ===
motor_body = chrono.ChBody()
motor_body.SetMass(0.01)
motor_body.SetInertiaXX(chrono.ChVector3d(1e-5, 1e-5, 1e-5))
motor_body.SetPos(arm_end)  # co-located at arm tip
sys.Add(motor_body)

# === Constraint 2 — arm tip node to motor body (rigid weld) ===
# ChLinkMateGeneric: all 6 DOF; links arm FEA node to the motor's rigid body
link_arm_motor = chrono.ChLinkMateGeneric()
link_arm_motor.Initialize(
    arm_nodes[-1],   # arm tip FEA node
    motor_body,      # motor attachment body
    False,
    arm_nodes[-1].Frame(),
    arm_nodes[-1].Frame(),
)
link_arm_motor.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(link_arm_motor)

# === Motor — custom angle function drives crank arm at arm tip ===
# ChLinkMotorRotationAngle is a FULL motor-link (no extra revolute needed).
# Motor rotation axis is Z (world), so QUNIT frame is correct for Z-axis pivot.
motor_frame = chrono.ChFramed(arm_end, chrono.QUNIT)
motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(motor_body, truss, motor_frame)
angle_fn = BucklingAngle()   # cache: keep strong reference to prevent GC
motor.SetAngleFunction(angle_fn)
sys.Add(motor)

# === FEA visualization shapes (two per mesh: surface field + glyph markers) ===

# Column — Shape 1: bending moment Mz (reveals beam curvature under buckling load)
vis_col_surf = chrono.ChVisualShapeFEA(mesh_col)
vis_col_surf.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_col_surf.SetColorscaleMinMax(-0.4, 0.4)
vis_col_surf.SetSmoothFaces(True)
vis_col_surf.SetWireframe(False)
mesh_col.AddVisualShapeFEA(vis_col_surf)

# Column — Shape 2: node coordinate-system glyphs
vis_col_glyph = chrono.ChVisualShapeFEA(mesh_col)
vis_col_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_col_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_col_glyph.SetSymbolsThickness(0.006)
vis_col_glyph.SetSymbolsScale(0.01)
vis_col_glyph.SetZbufferHide(False)
mesh_col.AddVisualShapeFEA(vis_col_glyph)

# Arm — Shape 3: bending moment Mz (shows load distribution in crank arm)
vis_arm_surf = chrono.ChVisualShapeFEA(mesh_arm)
vis_arm_surf.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_arm_surf.SetColorscaleMinMax(-0.4, 0.4)
vis_arm_surf.SetSmoothFaces(True)
vis_arm_surf.SetWireframe(False)
mesh_arm.AddVisualShapeFEA(vis_arm_surf)

# Arm — Shape 4: node position glyphs
vis_arm_glyph = chrono.ChVisualShapeFEA(mesh_arm)
vis_arm_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_arm_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_arm_glyph.SetSymbolsThickness(0.006)
vis_arm_glyph.SetSymbolsScale(0.01)
vis_arm_glyph.SetZbufferHide(False)
mesh_arm.AddVisualShapeFEA(vis_arm_glyph)

# === Visualization (Irrlicht — Initialize FIRST, then scene elements) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Beam Buckling — FEA Demo")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(1.2, 0.8, 1.8),  # eye — wide view of column + crank arm
    chrono.ChVector3d(0.0, 0.6, 0.0),  # target — column mid-height
)
vis.AddTypicalLights()

# === Review-only setup: frame recording ===


# === Main loop ===
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / bad integrator state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
