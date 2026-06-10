"""
Rotor FEA simulation — Jeffcott rotor with IGA beam and custom motor function.

The rotor shaft is modeled as an IGA Cosserat beam ( hollow circular cross-section).
A flywheel body is rigidly attached at the mid-span node. A ChLinkMotorRotationSpeed
drives the root node against a fixed truss via a piecewise custom speed function
(ChFunctionMyFun). This simulation demonstrates variable-speed rotor dynamics with
FEM elastic deformation.

System: ChSystemSMC (penalty-based smooth contact, appropriate for FEA).
"""

import math as m

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

# review-only: SIMBENCH_RECORD scaffolding


# === System & gravity ===
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === FEA mesh ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True, 2)   # 2 integration points per element for gravity precision
sys.Add(mesh)

# === Beam geometry & material ===
beam_L = 6.0
beam_ro = 0.050
beam_ri = 0.045
CH_PI = 3.1456

# Visual radius for rendering (larger so beam is visible in video)
VISUAL_BEAM_RO = 0.15

minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)
minertia.SetArea(CH_PI * (pow(beam_ro, 2) - pow(beam_ri, 2)))
minertia.SetIyy((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
minertia.SetIzz((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetIzz((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetJ((CH_PI / 2.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(VISUAL_BEAM_RO)

# === IGA beam construction ===
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(
    mesh,
    msection,
    20,   # number of spans
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(beam_L, 0, 0),
    chrono.VECT_Y,
    1,    # order: 1 = linear
)

# Cache the node container to avoid SWIG GC issues (must keep reference before indexing)
beam_nodes = builder.GetLastBeamNodes()  # cache: prevent SWIG GC of temporary container
node_mid = beam_nodes[m.floor(beam_nodes.size() / 2.0)]

# === Flywheel body (attached at mid-span node) ===
mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.24, 0.1, 7800)  # R, h, density
mbodyflywheel.SetCoordsys(
    chrono.ChCoordsysd(
        node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),
        chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Z),
    )
)
sys.Add(mbodyflywheel)

# Rigidly weld flywheel to mid-span node
weld = chrono.ChLinkMateFix()
weld.Initialize(node_mid, mbodyflywheel)
sys.Add(weld)

# === Fixed truss (ground reference) ===
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# === End bearing (constrains translations, allows rotation about X) ===
bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(
    beam_nodes.back(),
    truss,
    chrono.ChFramed(beam_nodes.back().GetPos()),
)
sys.Add(bearing)

# === Custom motor function class ===
class ChFunctionMyFun(chrono.ChFunction):
    """Piecewise motor speed profile.

    Parameters (hardcoded per spec):
        A1 = 0.8   (initial amplitude multiplier)
        A2 = 1.2   (final amplitude multiplier)
        T1 = 0.5   (ramp-up time)
        T2 = 1.0   (hold-start time)
        T3 = 1.25  (ramp-end time)
        w  = 60    (base angular speed rad/s)

    Speed profile:
        0 <= x < T1  : ramp up via cosine
        T1 <= x <= T2: constant at A1*w
        T2 < x <= T3 : ramp to A2*w via cosine
        x > T3       : constant at A2*w
    """

    def __init__(self):
        super().__init__()   # use super() to avoid SWIG director GC issue
        self._A1 = 0.8
        self._A2 = 1.2
        self._T1 = 0.5
        self._T2 = 1.0
        self._T3 = 1.25
        self._w = 60.0
        self._PI = 3.1456

    def Update(self, x):
        pass   # override empty to avoid SWIG director issue with GetVal

    def GetVal(self, x):
        A1 = self._A1
        A2 = self._A2
        T1 = self._T1
        T2 = self._T2
        T3 = self._T3
        w = self._w
        PI = self._PI
        if x < T1:
            return A1 * w * (1.0 - m.cos(PI * x / T1)) / 2.0
        elif x > T1 and x <= T2:
            return A1 * w
        elif x > T2 and x <= T3:
            return A1 * w + (A2 - A1) * w * (1.0 - m.cos(PI * (x - T2) / (T3 - T2))) / 2.0
        else:
            return A2 * w


# === Rotational motor (speed prescribed, full motor-link — no companion revolute) ===
# Keep motor_function alive for the entire simulation (prevents SWIG GC)
motor_function = ChFunctionMyFun()  # cache: module-level reference

rotmotor = chrono.ChLinkMotorRotationSpeed()
rotmotor.Initialize(
    beam_nodes.front(),   # slave: rotor root node
    truss,                # master: fixed truss
    chrono.ChFramed(
        beam_nodes.front().GetPos(),
        chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Y),
    ),
)
rotmotor.SetMotorFunction(motor_function)
sys.Add(rotmotor)

# === FEA mesh visualization (two-shape pattern) ===
vis_beam_surface = chrono.ChVisualShapeFEA()
vis_beam_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_beam_surface.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(vis_beam_surface)

vis_beam_glyph = chrono.ChVisualShapeFEA()
vis_beam_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_beam_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_beam_glyph.SetSymbolsThickness(0.006)
vis_beam_glyph.SetSymbolsScale(0.01)
vis_beam_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_beam_glyph)

# === Irrlicht visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Rotor FEA — Jeffcott rotor with custom motor function")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, 2, 5), chrono.ChVector3d(beam_L / 2.0, 0, 0))
vis.AddTypicalLights()

# Optional HHT timestepper (commented — off by default, enable if needed)
# sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)

# === Solver (Pardiso MKL for stiff beam FEA) ===
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# === Static pre-solve (settle structure under gravity / motor load) ===
sys.DoStaticLinear()

# === Simulation loop ===
time_step = 0.002   # IGA rotor: 2 ms timestep
sim_end = 3.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))


frame = 0
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
