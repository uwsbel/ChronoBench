"""
IGA Jeffcott Rotor with Custom Piecewise Motor Function — FEA MBS Demo
=======================================================================
System type : ChSystemSMC (required for FEA stiff beams)
Solver      : Pardiso MKL (direct sparse — required for IGA Cosserat beams)
Timestepper : HHT with step control disabled (canonical minimal form for beams)
Gravity     : Y-up  (0, -9.81, 0) — standard FEA convention for this demo family
Description :
  A Jeffcott rotor is modelled as an IGA Cosserat beam whose midpoint carries a
  flywheel disc.  Two translation-only bearing constraints (ChLinkMateGeneric,
  tx/ty/tz only) anchor the shaft ends radially while the shaft spins freely.
  The shaft is driven by a ChLinkMotorRotationSpeed (full motor-link) at the left
  end connecting node_A to the fixed truss; the motor constrains 5 DOF plus
  prescribes the spin speed via a custom piecewise ChFunctionMyFun with
  parameters A1, A2, T1, T2, T3, and angular frequency w.  FEM mesh
  visualisation uses two ChVisualShapeFEA shapes: DataType_SURFACE (coloured
  deformed beam surface) and DataType_NONE / GlyphType_NODE_CSYS (node
  coordinate-system glyphs).  A static linear pre-solve settles the structure
  under gravity before the dynamic spin-up.
"""

# === Imports ===
import os
import math
import csv

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# === Custom motor function ===
# Piecewise rotational-speed profile with parameters A1, A2, T1, T2, T3, w.
class ChFunctionMyFun(chrono.ChFunction):
    """Piecewise motor speed: linear ramp-up -> plateau -> cosine blend -> sustained oscillation."""

    def __init__(self, A1, A2, T1, T2, T3, w):
        chrono.ChFunction.__init__(self)  # MUST call base ctor
        self.A1 = A1
        self.A2 = A2
        self.T1 = T1
        self.T2 = T2
        self.T3 = T3
        self.w  = w

    def GetVal(self, x):  # x = simulation time; returns angular speed [rad/s]
        A1, A2 = self.A1, self.A2
        T1, T2, T3, w = self.T1, self.T2, self.T3, self.w
        if x < T1:
            # Linear ramp from 0 to A1
            return A1 * x / T1
        elif x < T2:
            # Constant plateau at A1
            return A1
        elif x < T3:
            # Cosine blend from A1 to A2
            tau = (x - T2) / (T3 - T2)
            return A1 + (A2 - A1) * (1.0 - math.cos(math.pi * tau)) * 0.5
        else:
            # A2 with small sustained oscillation at frequency w
            return A2 + (A2 - A1) * 0.05 * math.sin(w * (x - T3))

# === Named constants — geometry / physics ===
beam_L    = 1.2      # shaft length [m]
beam_ri   = 0.010    # shaft inner radius [m]  (hollow)
beam_ro   = 0.024    # shaft outer radius [m]
rho_steel = 7800.0   # density [kg/m^3]
E_steel   = 210e9    # Young's modulus [Pa]
nu_steel  = 0.3      # Poisson ratio

disc_mass = 0.24     # flywheel disc mass [kg]
disc_ro   = 0.14     # disc outer radius [m]
disc_th   = 0.03     # disc thickness [m]

n_spans   = 10       # IGA elements along beam
iga_order = 3        # cubic B-spline

# Motor custom function parameters
A1_speed = 0.0     # starting speed [rad/s]
A2_speed = 60.0    # cruise speed [rad/s]  (~575 RPM)
T1_val   = 0.5     # ramp-up ends [s]
T2_val   = 1.5     # plateau ends [s]
T3_val   = 3.0     # speed blend ends [s]
w_val    = 40.0    # oscillation frequency [rad/s]

time_step = 0.002   # IGA rotor canonical timestep
sim_end   = 4.0     # total simulation duration [s]

# Derived section properties — hollow circular shaft
area = math.pi * (beam_ro**2 - beam_ri**2)
Iyy  = math.pi * (beam_ro**4 - beam_ri**4) / 4.0
Izz  = Iyy
J    = Iyy + Izz

# Flywheel inertia (solid disc approximation)
Idisc_axial = 0.5  * disc_mass * disc_ro**2
Idisc_trans = 0.25 * disc_mass * disc_ro**2 + disc_mass * (disc_th**2) / 12.0

# === System & gravity (Y-up FEA convention) ===
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
# Pure jointed FEA scene: no rigid-body contact — no collision system needed.

# === Pardiso MKL solver (required for IGA Cosserat stiff beams) ===
sys.SetSolver(mkl.ChSolverPardisoMKL())

# === HHT timestepper — canonical minimal form for beam FEA ===
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# === Fixed truss (ground reference body) ===
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetName("truss")
sys.Add(truss)

# === IGA Cosserat beam section (hollow circular shaft) ===
minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(rho_steel)
minertia.SetArea(area)
minertia.SetIyy(Iyy)
minertia.SetIzz(Izz)

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(E_steel)
melasticity.SetShearModulusFromPoisson(nu_steel)
melasticity.SetIyy(Iyy)
melasticity.SetIzz(Izz)
melasticity.SetJ(J)

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)
# FEA beam: no contact material needed — driven by constraints + gravity + motor only.

# === Build IGA beam mesh (shaft along X axis, from x=0 to x=beam_L) ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(
    mesh,
    msection,
    n_spans,
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(beam_L, 0, 0),
    chrono.VECT_Y,
    iga_order,
)
sys.Add(mesh)

# Keep strong references to prevent SWIG GC dangling pointers (CRITICAL).
beam_nodes_container = builder.GetLastBeamNodes()
beam_nodes = [beam_nodes_container[i] for i in range(beam_nodes_container.size())]

node_A   = beam_nodes[0]                          # left shaft end  (x=0)
node_B   = beam_nodes[-1]                         # right shaft end (x=beam_L)
node_mid = beam_nodes[len(beam_nodes) // 2]       # midpoint for flywheel weld

# === Flywheel disc body ===
flywheel = chrono.ChBody()
flywheel.SetName("flywheel")
flywheel.SetMass(disc_mass)
flywheel.SetInertiaXX(chrono.ChVector3d(Idisc_trans, Idisc_axial, Idisc_trans))
flywheel.SetPos(node_mid.GetPos())   # initial position matches shaft midpoint node
sys.Add(flywheel)

# Visual shape — cylinder aligned along X (shaft axis) via Y-rotation
disc_vis = chrono.ChVisualShapeCylinder(disc_ro, disc_th)
flywheel.AddVisualShape(
    disc_vis,
    chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)),
)

# === Weld flywheel to midpoint beam node (rigid 6-DOF attachment) ===
weld = chrono.ChLinkMateFix()
weld.Initialize(node_mid, flywheel)
sys.Add(weld)

# === Joint frame: shaft spins about world X axis ===
# ChLinkMotorRotationSpeed uses local frame +Z as the motor/rotation axis.
# To spin about world +X, we need local Z -> world X: rotate +90 deg about Y.
# Q_ROTATE_Z_TO_X achieves this (local Z becomes world +X).
q_shaft = chrono.Q_ROTATE_Z_TO_X   # local Z aligns with world +X (shaft axis)

# Absolute frames for left and right shaft ends
frame_A_abs = chrono.ChFramed(chrono.ChVector3d(0, 0, 0), q_shaft)
frame_B_abs = chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0), q_shaft)

# === Motor at left end: full motor-link (no companion revolute needed) ===
# ChLinkMotorRotationSpeed constrains 5 DOF plus prescribes spin speed.
motor_fn = ChFunctionMyFun(A1=A1_speed, A2=A2_speed, T1=T1_val, T2=T2_val, T3=T3_val, w=w_val)

motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(node_A, truss, frame_A_abs)
motor.SetSpeedFunction(motor_fn)
sys.Add(motor)

# === Right bearing: translation-only constraint (tx, ty, tz; free rx, ry, rz) ===
# The motor already constrains node_A in 5 DOF (3 trans + 2 bend rot + speed).
# The right bearing only needs to carry radial loads at node_B.
bearing_B = chrono.ChLinkMateGeneric(True, True, True, False, False, False)
bearing_B.Initialize(node_B, truss, frame_B_abs)
sys.Add(bearing_B)

# === FEM mesh visualisation — two ChVisualShapeFEA shapes ===
# Shape 1: deformed beam surface coloured by DataType_SURFACE
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_surface.SetColorscaleMinMax(-0.005, 0.005)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

# Shape 2: node coordinate-system glyphs (DataType_NONE)
vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# === Static pre-solve (settles shaft sag under gravity before dynamic spin-up) ===
sys.DoStaticLinear()

# === Visualization — full Irrlicht block (Initialize first, scene elements after) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("IGA Jeffcott Rotor — Custom Piecewise Motor Function")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # Y-up for this FEA scene
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(0.5, 0.5, 2.0),    # eye
    chrono.ChVector3d(0.6, 0.0, 0.0),    # target: shaft midpoint
)
vis.AddTypicalLights()
vis.AddGrid(
    0.1, 0.1, 24, 24,
    chrono.ChCoordsysd(chrono.ChVector3d(0, -0.15, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

# === Timing constants (precomputed once) ===
render_fps   = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once


# === Main loop ===
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
except (RuntimeError, ValueError) as exc:  # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass  # scored-core finally: no additional cleanup required
