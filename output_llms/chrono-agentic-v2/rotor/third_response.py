"""
FEA IGA Rotor Simulation — Jeffcott rotor with custom piecewise motor function.

System: ChSystemSMC (required for FEA).
Structure: An IGA Cosserat beam shaft supported at both ends by bearings
  (ChLinkMateGeneric), with a flywheel disk welded to the shaft midpoint
  (ChLinkMateFix). The shaft is driven by a ChLinkMotorRotationSpeed whose
  speed is governed by a custom ChFunction subclass (ChFunctionMyFun) that
  implements a piecewise profile defined by parameters A1, A2, T1, T2, T3, w.
Visualization: FEA surface + glyph shapes via ChVisualShapeFEA; Irrlicht window.
Expected behaviour: The rotor spins up following the piecewise speed profile;
  the flexible shaft bends under gyroscopic + gravity loading.
"""

# === Imports ===
import os
import math
import csv

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

# === Constants — geometry & physics ===
beam_L   = 0.4       # shaft length (m)
beam_ro  = 0.0100    # shaft outer radius (m)
beam_ri  = 0.0000    # shaft inner radius (m, 0 = solid)
disk_mass = 0.01     # flywheel mass (kg)
disk_radius = 0.24   # flywheel radius (m)
disk_thick  = 0.05   # flywheel thickness (m)
density_steel = 7800.0  # kg/m³
E_steel = 210e9          # Young's modulus (Pa)
nu_steel = 0.3

n_spans = 10         # IGA beam spans (elements)
iga_order = 3        # cubic IGA

time_step = 0.002    # IGA rotor timestep (s)
sim_end   = 4.0      # simulation duration (s)
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# Cross-section properties — solid circular shaft
area = math.pi * beam_ro**2
Iyy  = math.pi * beam_ro**4 / 4.0
Izz  = Iyy
J    = math.pi * beam_ro**4 / 2.0

# === Custom motor function (piecewise speed profile) ===
# Parameters
A1 = 0.8    # speed level 1 (rad/s)
A2 = 1.2    # speed level 2 (rad/s)
T1 = 0.5    # end of ramp-up phase (s)
T2 = 1.5    # end of hold phase (s)
T3 = 3.0    # end of ramp-down / transition phase (s)
w  = 2.0    # ramp/oscillation scale factor (rad/s²)

class ChFunctionMyFun(chrono.ChFunction):
    """Piecewise rotational speed function for the shaft motor.

    Phase 1: 0 <= x < T1   — linear ramp from 0 to A1
    Phase 2: T1 <= x < T2  — hold at A1
    Phase 3: T2 <= x < T3  — sinusoidal transition from A1 to A2
    Phase 4: x >= T3        — hold at A2 with small sinusoidal ripple
    """
    def __init__(self):
        chrono.ChFunction.__init__(self)  # MUST call base ctor

    def GetVal(self, x):
        """Return motor speed (rad/s) at time x (s)."""
        if x < T1:
            return A1 * (x / T1)
        elif x < T2:
            return A1
        elif x < T3:
            t = (x - T2) / (T3 - T2)
            return A1 + (A2 - A1) * (1.0 - math.cos(math.pi * t)) / 2.0
        else:
            return A2 + 0.05 * A2 * math.sin(w * (x - T3))

# === System & gravity (FEA uses ChSystemSMC + Pardiso MKL) ===
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # Y-up convention
sys.SetSolver(mkl.ChSolverPardisoMKL())

# HHT timestepper (canonical-minimal form for IGA/beam FEA)
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# === FEA Mesh: IGA Cosserat beam (rotor shaft) ===
# FEA beam: no contact material needed — driven by constraints + gravity + motor only
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

# Inertia section
minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(density_steel)
minertia.SetArea(area)
minertia.SetIyy(Iyy)
minertia.SetIzz(Izz)

# Elasticity section
melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(E_steel)
melasticity.SetShearModulusFromPoisson(nu_steel)
melasticity.SetIyy(Iyy)
melasticity.SetIzz(Izz)
melasticity.SetJ(J)

# Combined Cosserat section
msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)  # SetAsCircularSection would overwrite Iyy/Izz/J

# Build IGA beam along X axis (Y-up world, shaft runs along X)
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(
    mesh,
    msection,
    n_spans,
    chrono.ChVector3d(0, 0, 0),        # start (bearing A)
    chrono.ChVector3d(beam_L, 0, 0),   # end   (bearing B)
    chrono.VECT_Y,                     # suggested section Y direction
    iga_order,
)

# SWIG GC safety — keep strong reference to node list
beam_nodes = builder.GetLastBeamNodes()
node_count = beam_nodes.size()

# Identify key nodes (keep strong references)
node_A   = beam_nodes[0]               # bearing A end
node_B   = beam_nodes[node_count - 1]  # bearing B end
node_mid = beam_nodes[node_count // 2] # flywheel attachment midpoint

sys.Add(mesh)

# === Bodies: fixed truss + flywheel disk ===
# Truss (ground)
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetName("truss")
sys.Add(truss)

# Flywheel disk welded at shaft midpoint
disk = chrono.ChBody()
disk.SetName("flywheel")
disk.SetPos(node_mid.GetPos())
# Disk inertia (solid cylinder about symmetry axis = X)
Iaxial = 0.5 * disk_mass * disk_radius**2
Itrans = disk_mass * (3.0 * disk_radius**2 + disk_thick**2) / 12.0
disk.SetMass(disk_mass)
disk.SetInertiaXX(chrono.ChVector3d(Iaxial, Itrans, Itrans))

# Disk visual shape (cylinder about local X)
cyl_shape = chrono.ChVisualShapeCylinder(disk_radius, disk_thick)
disk.AddVisualShape(cyl_shape,
    chrono.ChFramed(chrono.VNULL,
                    chrono.QuatFromAngleY(chrono.CH_PI_2)))
sys.Add(disk)

# === Constraints: bearings + flywheel weld ===
# Bearing A — pin shaft end node to truss (allow rotation about X, constrain translation + Y/Z rot)
bearing_A = chrono.ChLinkMateGeneric()
bearing_A.Initialize(node_A, truss, False,
                     node_A.Frame(),
                     node_A.Frame())
bearing_A.SetConstrainedCoords(True, True, True,   # tx, ty, tz
                                False, True, True) # rx(free), ry, rz
sys.Add(bearing_A)

# Bearing B — same: pin other end, free rotation about shaft axis (X)
bearing_B = chrono.ChLinkMateGeneric()
bearing_B.Initialize(node_B, truss, False,
                     node_B.Frame(),
                     node_B.Frame())
bearing_B.SetConstrainedCoords(True, True, True,
                                False, True, True)
sys.Add(bearing_B)

# Flywheel weld — fix disk rigidly to mid-shaft node
weld = chrono.ChLinkMateFix()
weld.Initialize(node_mid, disk)
sys.Add(weld)

# === Motor: custom piecewise speed function driving shaft about X axis ===
motor_fun = ChFunctionMyFun()  # cache: created once, referenced by motor
motor = chrono.ChLinkMotorRotationSpeed()
# Motor frame at node_A: local +Z must align with rotation axis (world X)
q_z_to_x = chrono.QuatFromAngleAxis(-chrono.CH_PI_2, chrono.VECT_Y)
motor.Initialize(node_A, truss,
                 chrono.ChFramed(node_A.GetPos(), q_z_to_x))
motor.SetSpeedFunction(motor_fun)
sys.Add(motor)

# Pre-solve static step to settle structure before dynamic run
sys.DoStaticLinear()

# === FEA Visualization — surface deformed shape + node glyphs ===
# Shape 1 — coloured surface (DataType_SURFACE shows deformed shape)
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_surface.SetColorscaleMinMax(-0.002, 0.002)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

# Shape 2 — node coordinate-system glyphs
vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# === Visualization: Irrlicht window ===
# (Initialize FIRST, then add scene elements — Irrlicht call order is inverse of VSG)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("FEA Rotor — Custom Motor Function")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)  # Y-up world
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, 0.3, 0.6),
              chrono.ChVector3d(beam_L / 2, 0, 0))  # AFTER Initialize
vis.AddTypicalLights()

# === CSV output setup (review-only) ===

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
except (RuntimeError, ValueError) as exc:  # solver divergence / bad beam state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
