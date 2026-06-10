"""
Jeffcott Rotor Simulation using IGA Beam (PyChrono FEA)
=======================================================
Models a flexible rotating shaft (Jeffcott rotor) using an Isogeometric Analysis
(IGA / Cosserat) beam element. A flywheel disk is rigidly welded to the mid-span
node. One end of the shaft is driven by a rotational speed motor; the other end
is constrained by a bearing. FEM visualization (surface + glyph) is rendered in
an Irrlicht window.

System type : ChSystemSMC
Solver      : Pardiso MKL (direct, required for stiff IGA beams)
Timestepper : HHT (canonical-minimal: SetStepControl False only)
Gravity     : Y-up, (0, -9.81, 0)
"""

import math
import os
import csv
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# FEA beam: no contact material needed — driven by constraints + gravity + motor only

# === Named constants ===
BEAM_L       = 2.0          # shaft length (m)
BEAM_RI      = 0.0           # inner radius (hollow: 0 = solid)
BEAM_RO      = 0.05          # outer radius of shaft (m)
N_SPANS      = 16            # number of IGA spans
IGA_ORDER    = 3             # cubic spline order

DENSITY      = 7800.0        # steel density (kg/m³)
YOUNG_MOD    = 210e9         # Young's modulus (Pa)
POISSON_NU   = 0.3           # Poisson ratio

FLYWHEEL_MASS   = 0.5        # flywheel disk mass (kg)
FLYWHEEL_RADIUS = 0.24       # flywheel radius (m)
FLYWHEEL_THICK  = 0.05       # flywheel thickness (m)

MOTOR_SPEED  = 60.0          # rotational speed (rad/s)
TIME_STEP    = 0.002         # IGA rotor recommended timestep
SIM_END      = 2.5           # simulation end time (s)
RENDER_FPS   = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Cross-section geometric properties (solid circular shaft)
AREA  = math.pi * BEAM_RO**2
IYY   = math.pi * BEAM_RO**4 / 4.0
IZZ   = IYY
J_POL = math.pi * BEAM_RO**4 / 2.0

# === System & gravity ===
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Solver & timestepper (Pardiso MKL + HHT canonical-minimal) ===
sys.SetSolver(mkl.ChSolverPardisoMKL())
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# === FEA Mesh — IGA Cosserat beam ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(False)   # gravity handled at system level

# IGA section: inertia + elasticity combined into Cosserat section
minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(DENSITY)
minertia.SetArea(AREA)
minertia.SetIyy(IYY)
minertia.SetIzz(IZZ)

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(YOUNG_MOD)
melasticity.SetShearModulusFromPoisson(POISSON_NU)
melasticity.SetIyy(IYY)
melasticity.SetIzz(IZZ)
melasticity.SetJ(J_POL)

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(BEAM_RO)

builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(
    mesh, msection, N_SPANS,
    chrono.ChVector3d(0, 0, 0),         # start A (x=0)
    chrono.ChVector3d(BEAM_L, 0, 0),    # end B (x=L)
    chrono.VECT_Y,                      # suggested section Y direction
    IGA_ORDER
)

# Retrieve beam nodes safely (keep strong ref to avoid SWIG GC pitfall)
beam_nodes_ref = builder.GetLastBeamNodes()
beam_nodes = [beam_nodes_ref[i] for i in range(beam_nodes_ref.size())]  # cache: strong refs

node_A   = beam_nodes[0]                           # driven end (x=0)
node_B   = beam_nodes[-1]                          # bearing end (x=L)
# Mid-span node for flywheel attachment
mid_idx  = len(beam_nodes) // 2
node_mid = beam_nodes[mid_idx]

sys.Add(mesh)

# === Ground truss (fixed reference body) ===
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# === Flywheel rigid body — welded to mid-span node ===
flywheel = chrono.ChBody()
flywheel.SetMass(FLYWHEEL_MASS)
Ixy = 0.5 * FLYWHEEL_MASS * FLYWHEEL_RADIUS**2   # disk: Ixx = Iyy = m*r²/4 approx; Izz = m*r²/2
flywheel.SetInertiaXX(chrono.ChVector3d(
    0.25 * FLYWHEEL_MASS * FLYWHEEL_RADIUS**2 + FLYWHEEL_MASS * FLYWHEEL_THICK**2 / 12.0,
    0.5  * FLYWHEEL_MASS * FLYWHEEL_RADIUS**2,
    0.25 * FLYWHEEL_MASS * FLYWHEEL_RADIUS**2 + FLYWHEEL_MASS * FLYWHEEL_THICK**2 / 12.0,
))
flywheel.SetPos(node_mid.GetPos())
sys.Add(flywheel)

# Add flywheel visual (cylinder disk aligned along shaft axis = X)
fw_shape = chrono.ChVisualShapeCylinder(FLYWHEEL_RADIUS, FLYWHEEL_THICK)
fw_shape.SetColor(chrono.ChColor(0.8, 0.3, 0.1))
flywheel.AddVisualShape(fw_shape,
    chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Weld flywheel to mid-span IGA node (ChLinkMateFix: 6 DOF rigid)
weld = chrono.ChLinkMateFix()
weld.Initialize(node_mid, flywheel)
sys.Add(weld)

# === Motor at node A — prescribes rotation speed (full motor-link) ===
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(
    node_A, truss,
    chrono.ChFramed(node_A.GetPos(),
                    chrono.QuatFromAngleY(chrono.CH_PI_2))   # local +Z → world +X (shaft axis)
)
motor.SetSpeedFunction(chrono.ChFunctionConst(MOTOR_SPEED))
sys.Add(motor)

# === Bearing at node B — ChLinkMateGeneric: constrain 5 DOF, free spin about X ===
bearing = chrono.ChLinkMateGeneric()
bearing.Initialize(node_B, truss, False,
                   node_B.Frame(), node_B.Frame())
bearing.SetConstrainedCoords(True, True, True,   # tx ty tz
                              False, True, True)  # rx=free (spin), ry rz constrained
sys.Add(bearing)

# === Pre-solve static step (settles structure before dynamic run) ===
sys.DoStaticLinear()

# === FEA Visualization shapes (attached before vis.Initialize()) ===
# Shape 1 — coloured surface showing displacement
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_surface.SetColorscaleMinMax(-0.002, 0.002)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

# Shape 2 — node-coordinate-system glyphs
vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# === Visualization — Irrlicht window (Initialize first, then scene elements) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Jeffcott Rotor — IGA Beam FEA")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(BEAM_L * 0.5, 0.5, 1.5),
              chrono.ChVector3d(BEAM_L * 0.5, 0.0, 0.0))
vis.AddTypicalLights()

# === Review-only recording setup ===


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
except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
