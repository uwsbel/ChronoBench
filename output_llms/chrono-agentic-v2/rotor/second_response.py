"""
Jeffcott Rotor simulation — FEA IGA beam shaft with a midspan flywheel.

System type: ChSystemSMC (required for FEA stiff beams).
Beam: hollow circular IGA/Cosserat beam, length=10, outer radius=0.060, inner radius=0.055.
Flywheel: rigid cylinder body (radius=0.30, height=0.1, steel density=7800) welded to the
    beam's midspan node via ChLinkMateFix.
Motor: ChLinkMotorRotationAngle driven by ChFunctionSine(60, 0.1) at one end bearing.
Gravity: Mars-like reduced gravity (0, -3.71, 0).
Two end bearings on a fixed truss constrain the shaft endpoints (ChLinkMateGeneric).
Expected behavior: beam spins up under the sine motor, flywheel oscillates/whirls due to
    unbalance (offset flywheel mass on flexible shaft), exhibiting Jeffcott rotor dynamics.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# === Physical constants ===
beam_L   = 10           # beam shaft length [m]
beam_ro  = 0.060        # outer radius [m]
beam_ri  = 0.055        # inner radius [m]
density  = 7800         # steel density [kg/m³]
E_steel  = 210e9        # Young's modulus [Pa]
nu_steel = 0.3          # Poisson ratio
beam_order = 3          # IGA cubic spline order
n_spans  = 10           # number of IGA spans

fw_radius = 0.30        # flywheel radius [m]  (changed from 0.24)
fw_height = 0.1         # flywheel height [m]
fw_density = 7800       # flywheel density [kg/m³]

time_step = 0.002       # integration step [s] — nominal for IGA rotor
sim_end   = 2.5         # simulation end time [s]
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# Cross-section derived geometry
area = math.pi * (beam_ro**2 - beam_ri**2)
Iyy  = math.pi / 4.0 * (beam_ro**4 - beam_ri**4)
Izz  = Iyy
J    = 2.0 * Iyy  # polar moment for circular hollow section

# === System & gravity ===
# ChSystemSMC required for FEA stiff IGA beams
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -3.71, 0))  # Mars-like gravity

# Pardiso MKL direct solver — required for stiff IGA/Cosserat beams
sys.SetSolver(mkl.ChSolverPardisoMKL())

# HHT timestepper (canonical-minimal form for stiff beams)
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# === FEA mesh & beam section ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

# IGA/Cosserat inertia properties
minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(density)
minertia.SetArea(area)
minertia.SetIyy(Iyy)
minertia.SetIzz(Izz)

# IGA/Cosserat elasticity properties
melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(E_steel)
melasticity.SetShearModulusFromPoisson(nu_steel)
melasticity.SetIyy(Iyy)
melasticity.SetIzz(Izz)
melasticity.SetJ(J)

# Assemble section
msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)  # visual radius; SetAsCircularSection would overwrite Iyy/Izz/J

# Build IGA beam from (0,0,0) to (beam_L, 0, 0)
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(
    mesh, msection,
    n_spans,
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(beam_L, 0, 0),
    chrono.VECT_Y,
    beam_order
)

# Keep strong ref to beam node list (SWIG GC pitfall)
beam_nodes = builder.GetLastBeamNodes()
n_nodes = beam_nodes.size()

# Mid-node index for flywheel attachment (integer division gives center span)
mid_idx  = n_nodes // 2
node_mid = beam_nodes[mid_idx]   # cache: used for flywheel weld

# End nodes for bearing constraints
node_A = beam_nodes[0]            # cache: left end bearing
node_B = beam_nodes[n_nodes - 1]  # cache: right end bearing

sys.Add(mesh)

# === Fixed truss (ground) ===
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetMass(1.0)
truss.SetInertiaXX(chrono.ChVector3d(1.0, 1.0, 1.0))
sys.Add(truss)

# === Flywheel body (rigid cylinder, radius=0.30) ===
mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, fw_radius, fw_height, fw_density)
mbodyflywheel.SetPos(node_mid.GetPos())
sys.Add(mbodyflywheel)

# FEA beam: no contact material needed — driven by constraints + gravity + motor only

# === Joints / constraints ===

# Left end bearing: constrain translation; allow rotation about X (shaft axis)
# Using ChLinkMateGeneric: fix tx,ty,tz and two of the rotations; free rx (spin axis)
bearing_A = chrono.ChLinkMateGeneric(True, True, True, False, True, True)
bearing_A.Initialize(
    node_A, truss, False,
    node_A.Frame(),
    node_A.Frame()
)
sys.Add(bearing_A)

# Right end bearing: same DOF removal as left
bearing_B = chrono.ChLinkMateGeneric(True, True, True, False, True, True)
bearing_B.Initialize(
    node_B, truss, False,
    node_B.Frame(),
    node_B.Frame()
)
sys.Add(bearing_B)

# Motor: ChLinkMotorRotationAngle at left bearing — Sine(60, 0.1)
# Full motor-link, no additional revolute needed
f_ramp = chrono.ChFunctionSine(60, 0.1)  # amplitude=60, frequency=0.1 Hz

motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(
    mbodyflywheel, truss,
    chrono.ChFramed(node_A.GetPos(), chrono.QuatFromAngleX(chrono.CH_PI_2))
)
motor.SetAngleFunction(f_ramp)
sys.Add(motor)

# Weld flywheel to midspan IGA node
weld = chrono.ChLinkMateFix()
weld.Initialize(node_mid, mbodyflywheel)
sys.Add(weld)

# Static linear pre-solve to settle gravity deformation
sys.DoStaticLinear()

# === FEA visualization shapes ===
# Shape 1 — deformed surface colored by Von Mises stress (or beam Mz)
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_surface.SetColorscaleMinMax(-0.4, 0.4)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

# Shape 2 — node glyphs
vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# === Visualization — full Irrlicht scene: window + sky + camera + lights ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Jeffcott Rotor — IGA beam, flywheel R=0.30, L=10")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()                  # Initialize FIRST (Irrlicht: inverse of VSG)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, 8), chrono.ChVector3d(beam_L / 2, 0, 0))  # camera at (0,2,8)
vis.AddTypicalLights()

# === Recording setup (review-only) ===


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
except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass  # scored core has no file handles to close
