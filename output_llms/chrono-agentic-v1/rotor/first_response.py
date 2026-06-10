"""
Jeffcott Rotor Simulation — PyChrono 9.0.0 / FEA / Irrlicht

Models a Jeffcott rotor as an IGA (Cosserat) beam element driven by a
rotational motor at one end. A flywheel (rigid disc) is welded to the
mid-span node via ChLinkMateFix. The system is ChSystemSMC with the
Pardiso MKL direct solver and HHT timestepper — the canonical setup for
stiff IGA beams. FEM visualization is provided by two ChVisualShapeFEA
shapes (surface + node glyphs). An Irrlicht window shows the dynamics.

World convention: Y-up. Gravity: (0, -9.81, 0).
No contact surfaces — this is a pure jointed FEA / motor scene.
# FEA beam: no contact material needed — driven by constraints + motor only.

Constraint topology:
  - Motor (ChLinkMotorRotationSpeed) at start node (x=0): full motor-link
    that spins the shaft about its own axis (X). Motor frame has +Z aligned
    to world +X so rotation is about the beam axis.
  - Bearing (ChLinkMateGeneric) at far end node (x=L): constrains tx, ty, tz
    only (translations), leaving all rotations free. This prevents over-
    constraining: the motor already constrains the end rotations at x=0.
  - Flywheel: rigid disc welded (ChLinkMateFix) to mid-span beam node.
"""

import math
import os
import csv
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl


# === Constants ===
# Beam geometry
BEAM_L       = 1.2          # shaft length [m]
BEAM_RO      = 0.020        # outer radius of hollow shaft [m]
BEAM_RI      = 0.015        # inner radius [m]
N_SPANS      = 4            # number of IGA span segments
IGA_ORDER    = 3            # cubic NURBS

# Material (steel)
DENSITY      = 7800.0       # kg/m³
YOUNG        = 210e9        # Pa
POISSON      = 0.3

# Flywheel
FW_RADIUS    = 0.12         # flywheel disc radius [m]
FW_THICKNESS = 0.025        # flywheel disc thickness [m]
FW_DENSITY   = 7800.0       # kg/m³

# Motor
MOTOR_SPEED  = 20.0         # rad/s  (~191 rpm) — reduced to avoid divergence

# Simulation
TIME_STEP    = 0.002        # s — canonical IGA rotor timestep
SIM_END      = 5.0          # s
RENDER_FPS   = 50.0         # frames per second for review video

# === Derived section properties (precomputed once) ===
area = math.pi * (BEAM_RO**2 - BEAM_RI**2)
Iyy  = math.pi / 4 * (BEAM_RO**4 - BEAM_RI**4)
Izz  = Iyy
J    = math.pi / 2 * (BEAM_RO**4 - BEAM_RI**4)

# Mass and inertia of flywheel disc
flywheel_mass = FW_DENSITY * math.pi * FW_RADIUS**2 * FW_THICKNESS  # precomputed once
flywheel_ixx  = 0.5 * flywheel_mass * FW_RADIUS**2
flywheel_iyy  = flywheel_mass * (3.0 * FW_RADIUS**2 + FW_THICKNESS**2) / 12.0
flywheel_izz  = flywheel_iyy

# Quaternion that rotates motor frame so its local +Z aligns with world +X
# (beam axis = X; motor prescribes rotation about X = local +Z after rotation)
# Rotate +Z → +X: rotate -90 deg about Y
q_motor_frame = chrono.QuatFromAngleY(-chrono.CH_PI_2)  # precomputed once

# render cadence
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# === System & gravity ===
# ChSystemSMC required for FEA; no SetCollisionSystemType for pure jointed FEA
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Solver + timestepper ===
# Pardiso MKL direct solver — required for stiff IGA beams
sys.SetSolver(mkl.ChSolverPardisoMKL())

# HHT timestepper — canonical-minimal form for beam/rotor FEA
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# === Fixed truss (ground body) ===
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetName("truss")
sys.Add(truss)

# === FEA mesh ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

# IGA (Cosserat) beam section — hollow circular cross-section
minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(DENSITY)
minertia.SetArea(area)
minertia.SetIyy(Iyy)
minertia.SetIzz(Izz)

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(YOUNG)
melasticity.SetShearModulusFromPoisson(POISSON)
melasticity.SetIyy(Iyy)
melasticity.SetIzz(Izz)
melasticity.SetJ(J)

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(BEAM_RO)

# Build the IGA beam along X-axis (Y-up world)
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(
    mesh, msection,
    N_SPANS,
    chrono.ChVector3d(0.0, 0.0, 0.0),    # start A (motor end)
    chrono.ChVector3d(BEAM_L, 0.0, 0.0),  # end B (bearing end)
    chrono.VECT_Y,                         # suggested section Y-up
    IGA_ORDER,
)

# Store strong references to prevent SWIG GC dangling pointers  (cache: strong refs)
beam_nodes_container = builder.GetLastBeamNodes()
beam_nodes = [beam_nodes_container[i] for i in range(beam_nodes_container.size())]

# Identify key nodes (cache: precomputed)
node_start = beam_nodes[0]          # x=0 — motor end
node_end   = beam_nodes[-1]         # x=L  — bearing end
node_mid   = min(beam_nodes, key=lambda n: abs(n.GetPos().x - BEAM_L / 2.0))  # closest to center

sys.Add(mesh)

# === Motor at the start end (x=0) ===
# ChLinkMotorRotationSpeed is a FULL motor-link that drives rotation about its local +Z.
# We rotate the frame so local +Z points along world +X (the beam axis).
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(
    node_start, truss,
    chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.0), q_motor_frame),
)
motor.SetSpeedFunction(chrono.ChFunctionConst(MOTOR_SPEED))
sys.Add(motor)

# === Bearing at the far end (x=L) ===
# Constrain only translations (tx, ty, tz); leave all rotations free.
# This avoids over-constraining with the motor which already handles rotations at x=0.
bearing_far = chrono.ChLinkMateGeneric(True, True, True, False, False, False)
bearing_far.Initialize(
    node_end, truss,
    False,
    node_end.Frame(),
    node_end.Frame(),
)
sys.Add(bearing_far)

# === Flywheel rigid disc welded to mid-span beam node ===
flywheel = chrono.ChBody()
flywheel.SetName("flywheel")
flywheel.SetMass(flywheel_mass)
flywheel.SetInertiaXX(chrono.ChVector3d(flywheel_ixx, flywheel_iyy, flywheel_izz))
flywheel.SetPos(node_mid.GetPos())
sys.Add(flywheel)

# Cylinder visual for the flywheel (disc in YZ plane, axis along X)
fw_vis = chrono.ChVisualShapeCylinder(FW_RADIUS, FW_THICKNESS)
flywheel.AddVisualShape(
    fw_vis,
    chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)),
)

# Weld flywheel to mid-span beam node (all 6 DOF locked)
weld = chrono.ChLinkMateFix()
weld.Initialize(node_mid, flywheel)
sys.Add(weld)

# === Static linear solve — settle structure before dynamic loop ===
sys.DoStaticLinear()

# === FEM visualization (attached to mesh before vis.Initialize()) ===
# Shape 1 — surface field (deformed shape coloured by surface data)
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_surface.SetColorscaleMinMax(-0.01, 0.01)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

# Shape 2 — node glyphs (coordinate-system triads at every node)
vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# === Visualization — full Irrlicht block (Initialize first, scene elements after) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Jeffcott Rotor — IGA Beam FEA")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()                                       # FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(BEAM_L * 0.5, 0.6, 1.5),        # eye — above and to the side
    chrono.ChVector3d(BEAM_L * 0.5, 0.0, 0.0),         # target — beam center
)
vis.AddTypicalLights()
vis.AddGrid(
    0.1, 0.1, 20, 20,
    chrono.ChCoordsysd(chrono.ChVector3d(0, -FW_RADIUS - 0.05, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

# === Main loop ===


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
except (RuntimeError, ValueError) as exc:
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
