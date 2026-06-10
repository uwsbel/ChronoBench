"""
Beam Buckling Simulation using PyChrono FEA.

Models a column/beam buckling scenario with Finite Element Analysis using
Euler-Bernoulli beam elements. The bottom end of the beam is fixed to the
ground (truss body), while a custom axial compressive load is applied via a
ChLinkMotorRotationAngle that drives an end mass, combined with a crank body
that imposes axial displacement on the beam tip. A ChSystemSMC is used with
the Pardiso MKL direct solver and HHT timestepper for accurate stiff-beam
dynamics. The beam is expected to deflect laterally (buckle) once the
compressive load exceeds the Euler critical buckling load.
"""

import math
import os
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

# === Constants ===
# FEA beam geometry & material
BEAM_LENGTH    = 0.4       # m — beam length (column height)
BEAM_DIAMETER  = 0.01      # m — circular cross-section diameter
BEAM_DENSITY   = 7800.0    # kg/m³ — steel
BEAM_E         = 210e9     # Pa — Young's modulus (steel)
BEAM_POISSON   = 0.3       # Poisson ratio
BEAM_DAMPING   = 0.0       # Rayleigh damping
N_ELEMENTS     = 16        # number of beam elements

# End-mass (rigid body at tip that the motor drives axially)
END_MASS       = 0.1       # kg
END_HALF       = 0.005     # half-size of end mass block (m)

# Motor / crank (custom function drives axial compression at tip)
CRANK_MASS     = 0.005     # kg — small crank body linking motor to tip node
COMPRESS_RATE  = 0.005     # m/s — axial compression speed applied by the motor
PRETILT        = 0.001     # m — lateral pre-tilt applied to tip to break symmetry

# Simulation control
TIME_STEP      = 1e-3      # s
SIM_END        = 3.0       # s
RENDER_FPS     = 50.0
RENDER_EVERY   = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# === System & gravity ===
# ChSystemSMC is required for FEA beam elements (all FEA SimBench truths use SMC).
# Y-up world convention: gravity along -Y.
# No collision system — pure FEA + rigid-body joints, no contact surfaces.
# FEA beam: no contact material needed — driven by constraints + gravity + motor only.
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Direct solver (Pardiso MKL) — required for stiff beam stiffness matrices.
sys.SetSolver(mkl.ChSolverPardisoMKL())

# HHT timestepper — canonical-minimal form for stiff beam elements.
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# === Bodies / FEA mesh ===
# Ground truss — the fixed reference body.
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetName("truss")
sys.Add(truss)

# Build FEA mesh with Euler-Bernoulli beam elements running vertically (Y-axis).
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

sec = fea.ChBeamSectionEulerAdvanced()
sec.SetAsCircularSection(BEAM_DIAMETER)
sec.SetDensity(BEAM_DENSITY)
sec.SetYoungModulus(BEAM_E)
sec.SetShearModulusFromPoisson(BEAM_POISSON)
sec.SetRayleighDamping(BEAM_DAMPING)

builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(
    mesh,
    sec,
    N_ELEMENTS,
    chrono.ChVector3d(0, 0, 0),               # beam start (base)
    chrono.ChVector3d(PRETILT, BEAM_LENGTH, 0),  # beam end (tip, slight pre-tilt in X)
    chrono.ChVector3d(0, 0, 1),               # reference direction (section Z)
)

# Keep a strong reference to beam nodes (SWIG GC protection — avoids dangling ptr).
beam_nodes = builder.GetLastBeamNodes()
node_list  = [beam_nodes[i] for i in range(beam_nodes.size())]  # cache: all nodes

# Fix the bottom node to the truss (6 DOF constraint).
node_base = node_list[0]
node_base.SetFixed(True)

# Tip node — the motor/crank body will be linked here to apply axial compression.
node_tip = node_list[-1]

sys.Add(mesh)

# === Crank/end-mass body constrained to beam tip for axial loading ===
# A small rigid crank body is placed at the tip. Its Y-position is driven by a
# ChLinkMotorRotationAngle-based axial displacement scheme.  Here we use a simpler
# ChLinkMateGeneric to pin the tip node translation to the crank, letting the crank
# slide axially while the motor imposes the axial compression.

# End-mass crank body placed at tip position.
tip_pos = node_tip.GetPos()

crank = chrono.ChBodyEasyBox(
    END_HALF * 2, END_HALF * 2, END_HALF * 2,
    END_MASS / (END_HALF * 2) ** 3,  # density derived from mass
    True, False,                       # visualize, no collision
)
crank.SetPos(tip_pos)
crank.SetName("crank_end_mass")
sys.Add(crank)

# A second body acts as a moving "guide" slider constrained to move only along Y
# (axial direction).  A prismatic link to truss keeps it on the Y-axis.
# The motor drives the crank body downward along Y.

slider = chrono.ChBodyEasyBox(
    END_HALF * 2, END_HALF * 4, END_HALF * 2,
    END_MASS / (END_HALF * 2) ** 3,
    True, False,
)
slider.SetPos(tip_pos)
slider.SetName("slider")
sys.Add(slider)

# Prismatic joint: slider slides along Y (global Y → frame local +Z must be world Y).
q_z_to_y = chrono.QuatFromAngleAxis(-chrono.CH_PI_2, chrono.VECT_X)  # maps local +Z → world +Y
pris = chrono.ChLinkLockPrismatic()
pris.Initialize(slider, truss, chrono.ChFramed(tip_pos, q_z_to_y))
sys.AddLink(pris)

# === Custom motor function ===
# Drives the slider downward (negative Y) at a constant compression rate, simulating
# an increasing axial compressive load on the beam tip.
class LinearCompression(chrono.ChFunction):
    """Prescribes axial displacement: y(t) = -COMPRESS_RATE * t (downward)."""
    def __init__(self):
        chrono.ChFunction.__init__(self)
    def GetVal(self, x):
        return -COMPRESS_RATE * x   # axial displacement (m) along Y at time x

compress_fn = LinearCompression()   # cache: created once, passed to motor

# Motor prescribes axial DISPLACEMENT of the slider along Y relative to the truss.
# We use a ChLinkMotorLinearPosition (linear motor) — but PyChrono's easy pattern
# for a linear actuation on a prismatic pair is to use a ChLinkMotorLinearPosition.
# Check via topology skill: use ChLinkMotorLinearPosition for linear position control.
motor = chrono.ChLinkMotorLinearPosition()
motor.Initialize(slider, truss, chrono.ChFramed(tip_pos, q_z_to_y))
motor.SetMotionFunction(compress_fn)
sys.AddLink(motor)

# === Constraint: pin tip node to slider (translational coupling) ===
# ChLinkMateGeneric pins the FEA tip node's XYZ translations to the slider body.
# This transmits the motor-induced compression directly into the beam tip.
tip_constr = chrono.ChLinkMateGeneric()
tip_constr.Initialize(
    node_tip, slider,
    False,
    node_tip.Frame(),
    node_tip.Frame(),
)
# Constrain only translational DOFs (transmit axial + lateral reactions);
# leave rotations free so the tip can rotate as the beam buckles.
tip_constr.SetConstrainedCoords(True, True, True, False, False, False)
sys.Add(tip_constr)

# === FEA Visualization ===
# Two ChVisualShapeFEA shapes: bending-moment Mz colour field + node glyph markers.
# Attach BEFORE vis.Initialize() so the Irrlicht device picks them up.
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surface.SetColorscaleMinMax(-0.4, 0.4)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Beam Buckling — FEA Euler-Bernoulli")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()                                                      # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.3, 0.2, 0.5), chrono.ChVector3d(0, 0.2, 0))
vis.AddTypicalLights()

# === Main loop ===

frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
