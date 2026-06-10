"""Motor-compressed FEA beam buckling simulation using PyChrono SMC.

The model builds a vertical Euler-Bernoulli beam between a fixed base platen and
a guided top platen. A custom ChFunction drives a linear motor that shortens the
column, while node/body constraints transfer the platen motion into the FEA mesh.
The expected response is lateral buckling under axial compression, rendered with
Irrlicht and integrated with Pardiso MKL plus an HHT timestepper.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl


# === Constants === beam, motor, and render constants kept explicit
BEAM_LENGTH = 2.0
BEAM_DIAMETER = 0.030
BEAM_ELEMENTS = 18
BEAM_DENSITY = 7850.0
YOUNG_MODULUS = 2.0e7
POISSON_RATIO = 0.30
RAYLEIGH_DAMPING = 0.002
TOP_PLATEN_MASS = 3.0
TOP_PLATEN_INERTIA = 0.02
PLATEN_SIZE_X = 0.35
PLATEN_SIZE_Y = 0.045
PLATEN_SIZE_Z = 0.35
COMPRESSION_DISTANCE = 0.45
COMPRESSION_TIME = 1.8
LATERAL_FORCE = 45.0
MAX_COMPRESSION_LOAD = 1300.0
TIME_STEP = 0.001
SIM_END = 3.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once


class SmoothCompression(chrono.ChFunction):
    """Custom motor function: smooth downward displacement over COMPRESSION_TIME."""

    def __init__(self, travel, duration):
        chrono.ChFunction.__init__(self)
        self.travel = travel
        self.duration = duration

    def GetVal(self, x):
        if x <= 0.0:
            return 0.0
        if x >= self.duration:
            return -self.travel
        phase = x / self.duration
        return -self.travel * (0.5 - 0.5 * math.cos(math.pi * phase))


def make_visual_box(name, pos, size, fixed, mass, color):
    """Create a rigid visual-only platen or guide body."""
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(fixed)
    body.SetMass(mass)
    body.SetInertiaXX(chrono.ChVector3d(TOP_PLATEN_INERTIA, TOP_PLATEN_INERTIA, TOP_PLATEN_INERTIA))
    body.SetPos(pos)
    shape = chrono.ChVisualShapeBox(size)
    shape.SetColor(color)
    body.AddVisualShape(shape)
    return body


# === System & gravity === SMC FEA system with direct solver and HHT integration
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
sys.SetSolver(mkl.ChSolverPardisoMKL())
timestepper = chrono.ChTimestepperHHT(sys)
timestepper.SetStepControl(False)
sys.SetTimestepper(timestepper)


# === FEA beam === Euler beam column with strong references kept for SWIG safety
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(False)

section = fea.ChBeamSectionEulerAdvanced()
section.SetAsCircularSection(BEAM_DIAMETER)
section.SetDensity(BEAM_DENSITY)
section.SetYoungModulus(YOUNG_MODULUS)
section.SetShearModulusFromPoisson(POISSON_RATIO)
section.SetRayleighDamping(RAYLEIGH_DAMPING)

builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(
    mesh,
    section,
    BEAM_ELEMENTS,
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(0, BEAM_LENGTH, 0),
    chrono.VECT_Z,
)
beam_node_vector = builder.GetLastBeamNodes()  # cache: SWIG vector retained for node lifetime
beam_nodes = [beam_node_vector[i] for i in range(beam_node_vector.size())]  # cache: reused in loop
bottom_node = beam_nodes[0]  # cache: constrained to base
mid_node = beam_nodes[len(beam_nodes) // 2]  # cache: receives lateral imperfection load
top_node = beam_nodes[-1]  # cache: constrained to moving platen

vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surface.SetColorscaleMinMax(-2.0, 2.0)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.018)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

sys.Add(mesh)
# FEA beam: no contact material needed, because it is driven by constraints and a motor only.


# === Rigid platens and constraints === base, guided top platen, node/body links, and motor
ground = make_visual_box(
    "fixed_base_platen",
    chrono.ChVector3d(0, -PLATEN_SIZE_Y * 0.5, 0),
    chrono.ChVector3d(PLATEN_SIZE_X, PLATEN_SIZE_Y, PLATEN_SIZE_Z),
    True,
    1.0,
    chrono.ChColor(0.25, 0.30, 0.34),
)
top_platen = make_visual_box(
    "motorized_top_platen",
    chrono.ChVector3d(0, BEAM_LENGTH, 0),
    chrono.ChVector3d(PLATEN_SIZE_X, PLATEN_SIZE_Y, PLATEN_SIZE_Z),
    False,
    TOP_PLATEN_MASS,
    chrono.ChColor(0.70, 0.18, 0.14),
)
sys.AddBody(ground)
sys.AddBody(top_platen)

base_clamp = chrono.ChLinkMateGeneric()
base_clamp.Initialize(bottom_node, ground, False, bottom_node.Frame(), bottom_node.Frame())
base_clamp.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(base_clamp)

top_weld = chrono.ChLinkMateFix()
top_weld.Initialize(top_node, top_platen)
sys.Add(top_weld)

motor_frame = chrono.ChFramed(chrono.ChVector3d(0, BEAM_LENGTH, 0), chrono.Q_ROTATE_X_TO_Y)
compression_motor = chrono.ChLinkMotorLinearPosition()
compression_motor.Initialize(top_platen, ground, motor_frame)
compression_motor.SetGuideConstraint(chrono.ChLinkMotorLinear.GuideConstraint_PRISMATIC)
compression_profile = SmoothCompression(COMPRESSION_DISTANCE, COMPRESSION_TIME)  # cache: keep Python ChFunction alive for SWIG director
compression_motor.SetMotionFunction(compression_profile)
sys.Add(compression_motor)


# === Visualization === Irrlicht window, sky, camera, lights, and reference grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("FEA Beam Buckling")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2.4, 1.1, 2.8), chrono.ChVector3d(0, 0.9, 0))
vis.AddTypicalLights()
vis.AddGrid(
    0.25,
    0.25,
    16,
    16,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.Q_ROTATE_Z_TO_Y),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main loop === render once per frame and step FEA in fixed-size batches
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            time_now = sys.GetChTime()  # cache: reused for logging and load ramp
            perturb_scale = min(1.0, time_now / 0.4)
            load_scale = abs(compression_profile.GetVal(time_now)) / COMPRESSION_DISTANCE
            mid_node.SetForce(chrono.ChVector3d(LATERAL_FORCE * perturb_scale, 0, 0))
            top_node.SetForce(chrono.ChVector3d(LATERAL_FORCE * 0.35 * perturb_scale, -MAX_COMPRESSION_LOAD * load_scale, 0))
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence or invalid state
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:  # runtime environment failure
    traceback.print_exc()
    raise
finally:
    pass
