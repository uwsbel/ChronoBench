"""
Jeffcott rotor simulation using IGA Cosserat beam elements (ChSystemSMC).
The flexible shaft is built with n_spans IGA spans. A flywheel body is welded to
the mid-span node. Both shaft ends are pinned to the fixed truss with
ChLinkNodeFrame constraints (xyz constrained, rotations free).
A ChLinkMotorRotationSpeed drives the flywheel against the truss using a custom
ChFunctionMyFun that varies speed piecewise over time via parameters A1, A2, T1, T2, T3, w.
FEA visualization: DataType_SURFACE (deformed beam) + DataType_NONE (node glyphs).
Expected: shaft deflects laterally from centrifugal unbalance; motor speed ramps A1→A2.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

# === Constants ===
time_step  = 0.002        # s — IGA rotor timestep
sim_end    = 10.0         # s
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# Beam geometry
beam_L  = 1.2    # m — shaft length (X axis)
beam_ro = 0.010  # m — outer shaft radius
beam_ri = 0.006  # m — inner shaft radius
n_spans = 12     # IGA spans along shaft
order   = 3      # cubic IGA

# Steel material
density = 7800.0   # kg/m³
young_E = 210e9    # Pa
poisson = 0.3

# Section derived properties (hollow circle); precomputed once
area = math.pi * (beam_ro**2 - beam_ri**2)
Iyy  = math.pi * (beam_ro**4 - beam_ri**4) / 4.0
Izz  = Iyy
J    = 2.0 * Iyy

# Flywheel
flywheel_mass   = 0.5    # kg
flywheel_radius = 0.12   # m
flywheel_offset = 0.002  # m eccentricity for unbalance

# Custom motor function parameters
A1 = 0.8   # speed level 1 (rev/s)
A2 = 1.2   # speed level 2 (rev/s)
T1 = 2.0   # s — end of ramp phase
T2 = 5.0   # s — end of plateau phase
T3 = 8.0   # s — end of blend phase
w  = 4.0   # rad/s — blend angular frequency


# === Custom motor function (piecewise varying speed) ===
class ChFunctionMyFun(chrono.ChFunction):
    """Piecewise rotational speed (rad/s) as a function of time x.
    [0, T1)   : linear ramp   0 → A1*2pi
    [T1, T2)  : constant      A1*2pi
    [T2, T3)  : cosine blend  A1 → A2 (using parameter w)
    [T3, inf) : constant      A2*2pi
    """
    def __init__(self):
        chrono.ChFunction.__init__(self)   # MUST call base ctor

    def Clone(self):
        return ChFunctionMyFun()

    def GetVal(self, x):
        if x < T1:
            return A1 * (x / T1) * 2.0 * math.pi
        elif x < T2:
            return A1 * 2.0 * math.pi
        elif x < T3:
            blend = (x - T2) / (T3 - T2)
            return (A1 + (A2 - A1) * 0.5 * (1.0 - math.cos(math.pi * blend))) * 2.0 * math.pi
        else:
            return A2 * 2.0 * math.pi


# === System & solver ===
# ChSystemSMC for FEA; Pardiso MKL for stiff IGA beams; Y-up
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
# Pure jointed FEA (no rigid-body contact) — SetCollisionSystemType not required

sys.SetSolver(mkl.ChSolverPardisoMKL())

ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)


# === Bodies ===
# Fixed truss — ground reference for bearings and motor
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetName("truss")
sys.Add(truss)

# Flywheel disc body (welded to mid-span beam node)
Ixx_fw = 0.25 * flywheel_mass * flywheel_radius**2
Iyy_fw = 0.5  * flywheel_mass * flywheel_radius**2
flywheel = chrono.ChBody()
flywheel.SetMass(flywheel_mass)
flywheel.SetInertiaXX(chrono.ChVector3d(Ixx_fw, Iyy_fw, Ixx_fw))
flywheel.SetPos(chrono.ChVector3d(beam_L / 2.0, flywheel_offset, 0.0))
flywheel.SetName("flywheel")
sys.Add(flywheel)

vis_fw = chrono.ChVisualShapeCylinder(flywheel_radius, 0.02)
flywheel.AddVisualShape(vis_fw,
    chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleZ(chrono.CH_PI_2)))


# === FEA mesh (IGA Cosserat shaft) ===
# FEA beam: no contact material needed — driven by constraints + motor only
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(density)
minertia.SetArea(area)
minertia.SetIyy(Iyy)
minertia.SetIzz(Izz)

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(young_E)
melasticity.SetShearModulusFromPoisson(poisson)
melasticity.SetIyy(Iyy)
melasticity.SetIzz(Izz)
melasticity.SetJ(J)

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)  # avoids overwriting Iyy/Izz/J

builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(
    mesh, msection, n_spans,
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(beam_L, 0, 0),
    chrono.VECT_Y,
    order,
)

# Cache node list (SWIG GC pitfall — strong ref prevents dangling pointers)
beam_nodes = builder.GetLastBeamNodes()  # cache: strong ref
n_nodes    = beam_nodes.size()
node_A     = beam_nodes[0]              # left end
node_B     = beam_nodes[n_nodes - 1]   # right end
node_mid   = beam_nodes[n_nodes // 2]  # mid-span (flywheel)

sys.Add(mesh)


# === Joints / constraints ===
# Pin both beam ends to truss: constrain tx,ty,tz; leave rx,ry,rz free (journal bearings)
# IGA nodes are ChNodeFEAxyzrot — use ChLinkMateGeneric (ChLinkNodeFrame requires xyz only)
pin_A = chrono.ChLinkMateGeneric(True, True, True, False, False, False)
pin_A.Initialize(node_A, truss, False, node_A.Frame(), node_A.Frame())
sys.Add(pin_A)

pin_B = chrono.ChLinkMateGeneric(True, True, True, False, False, False)
pin_B.Initialize(node_B, truss, False, node_B.Frame(), node_B.Frame())
sys.Add(pin_B)

# Weld flywheel disc to mid-span beam node (rigid)
weld = chrono.ChLinkMateFix()
weld.Initialize(node_mid, flywheel)
sys.Add(weld)

# Motor: prescribed rotational speed on flywheel body vs truss
# ChLinkMotorRotationSpeed is a full motor-link (no separate revolute needed).
# Rotation axis = world X (beam axis). Frame: QuatFromAngleAxis(+pi/2, Y) maps local Z → world X.
my_speed_fun = ChFunctionMyFun()   # keep strong reference to prevent GC
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(
    flywheel, truss,
    chrono.ChFramed(
        chrono.ChVector3d(beam_L / 2.0, flywheel_offset, 0.0),
        chrono.QuatFromAngleAxis(chrono.CH_PI_2, chrono.VECT_Y)
    )
)
motor.SetSpeedFunction(my_speed_fun)
sys.Add(motor)


# === FEA visualization shapes (attach BEFORE vis.Initialize) ===
# Shape 1 — deformed beam surface
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_surface.SetColorscaleMinMax(-0.004, 0.004)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

# Shape 2 — node glyphs (DataType_NONE)
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
vis.SetWindowTitle("Jeffcott Rotor — ChFunctionMyFun Speed Control")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(beam_L * 0.5, 0.4, 0.9),
    chrono.ChVector3d(beam_L * 0.5, 0.0, 0.0)
)
vis.AddTypicalLights()


# === Review-only recording setup ===

# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass   # CSV closed in review-only block below
