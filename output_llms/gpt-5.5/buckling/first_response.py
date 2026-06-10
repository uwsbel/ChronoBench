# beam_buckling_fea.py
#
# PyChrono FEA beam-buckling example:
# - Euler beam finite elements
# - motor-driven axial compression
# - node/body constraints at both beam ends
# - custom motor displacement function
# - small lateral perturbation to trigger buckling
# - Irrlicht visualization
# - MINRES solver + HHT timestepper

import math
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


# -----------------------------------------------------------------------------
# Compatibility helpers for different PyChrono versions
# -----------------------------------------------------------------------------
try:
    Vec = chrono.ChVector3d
    Frame = chrono.ChFramed
except AttributeError:
    Vec = chrono.ChVectorD
    Frame = chrono.ChFrameD


def set_gravity(system, g):
    if hasattr(system, "SetGravitationalAcceleration"):
        system.SetGravitationalAcceleration(g)
    else:
        system.Set_G_acc(g)


# -----------------------------------------------------------------------------
# Custom motor function: smooth ramp compression
# -----------------------------------------------------------------------------
class SmoothCompressionFunction(chrono.ChFunction):
    """
    Returns an imposed axial displacement for the linear motor.

    The displacement starts at zero, smoothly ramps to -max_disp, then holds.
    Negative displacement compresses the beam along the global X direction.
    """

    def __init__(self, max_disp=0.015, ramp_time=2.0):
        super().__init__()
        self.max_disp = max_disp
        self.ramp_time = ramp_time

    def GetVal(self, t):
        if t <= 0:
            return 0.0
        if t >= self.ramp_time:
            return -self.max_disp

        # Smoothstep interpolation: 3s^2 - 2s^3
        s = t / self.ramp_time
        smooth = 3.0 * s * s - 2.0 * s * s * s
        return -self.max_disp * smooth

    def Clone(self):
        return SmoothCompressionFunction(self.max_disp, self.ramp_time)


class LateralPerturbationFunction(chrono.ChFunction):
    """
    Small lateral force pulse to break perfect symmetry and initiate buckling.
    """

    def __init__(self, force=0.15, duration=0.15):
        super().__init__()
        self.force = force
        self.duration = duration

    def GetVal(self, t):
        if 0.0 <= t <= self.duration:
            return self.force * math.sin(math.pi * t / self.duration)
        return 0.0

    def Clone(self):
        return LateralPerturbationFunction(self.force, self.duration)


# -----------------------------------------------------------------------------
# Create Chrono system
# -----------------------------------------------------------------------------
system = chrono.ChSystemSMC()
set_gravity(system, Vec(0, 0, 0))  # buckling from axial compression, not gravity


# -----------------------------------------------------------------------------
# FEA beam mesh
# -----------------------------------------------------------------------------
mesh = fea.ChMesh()
system.Add(mesh)

# Beam geometry and material
beam_length = 1.0
beam_width = 0.012
beam_height = 0.006
num_elements = 24

young_modulus = 2.1e11       # steel-like
poisson_ratio = 0.30
density = 7800.0

# Euler-Bernoulli beam section
section = fea.ChBeamSectionEulerAdvanced()
section.SetAsRectangularSection(beam_width, beam_height)
section.SetYoungModulus(young_modulus)
section.SetGwithPoissonRatio(poisson_ratio)
section.SetDensity(density)

# Small structural damping
if hasattr(section, "SetBeamRaleyghDamping"):
    section.SetBeamRaleyghDamping(0.002)

# Build beam along global X, with local Y approximately global Y
builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(
    mesh,
    section,
    num_elements,
    Vec(0, 0, 0),
    Vec(beam_length, 0, 0),
    Vec(0, 1, 0),
)

beam_nodes = builder.GetLastBeamNodes()

# Add tiny geometric imperfection to help trigger a visible buckling mode.
# This represents a real-world non-perfectly-straight column.
imperfection_amp = 0.001
for node in beam_nodes:
    p = node.GetPos()
    x = p.x if hasattr(p, "x") else p[0]
    y_imp = imperfection_amp * math.sin(math.pi * x / beam_length)
    node.SetPos(Vec(x, y_imp, 0))


# -----------------------------------------------------------------------------
# Rigid clamps and constraints
# -----------------------------------------------------------------------------
ground = chrono.ChBody()
ground.SetFixed(True)
system.Add(ground)

clamp_size_x = 0.025
clamp_size_y = 0.055
clamp_size_z = 0.055

left_clamp = chrono.ChBodyEasyBox(
    clamp_size_x, clamp_size_y, clamp_size_z, 1000, True, False
)
left_clamp.SetPos(Vec(0, 0, 0))
left_clamp.SetFixed(True)
system.Add(left_clamp)

right_clamp = chrono.ChBodyEasyBox(
    clamp_size_x, clamp_size_y, clamp_size_z, 1000, True, False
)
right_clamp.SetPos(Vec(beam_length, 0, 0))
right_clamp.SetFixed(False)
system.Add(right_clamp)

# Constrain beam end nodes to rigid clamp bodies.
# ChLinkNodeFrame gives a frame-type constraint when available.
# If unavailable, fall back to point-frame constraints.
def add_node_clamp_constraint(node, body):
    try:
        link = fea.ChLinkNodeFrame()
        link.Initialize(node, body)
    except Exception:
        link = fea.ChLinkPointFrame()
        link.Initialize(node, body)
    system.Add(link)
    return link


left_node_constraint = add_node_clamp_constraint(beam_nodes[0], left_clamp)
right_node_constraint = add_node_clamp_constraint(beam_nodes[-1], right_clamp)


# -----------------------------------------------------------------------------
# Motor-driven axial compression
# -----------------------------------------------------------------------------
compression_function = SmoothCompressionFunction(
    max_disp=0.020,   # total imposed end shortening
    ramp_time=2.5,
)

motor = chrono.ChLinkMotorLinearPosition()

# Motor frame axis is along global X by default.
# It drives the right clamp relative to ground.
motor.Initialize(
    right_clamp,
    ground,
    Frame(Vec(beam_length, 0, 0))
)

if hasattr(motor, "SetMotionFunction"):
    motor.SetMotionFunction(compression_function)
else:
    motor.SetMotorFunction(compression_function)

system.Add(motor)


# -----------------------------------------------------------------------------
# FEA visualization
# -----------------------------------------------------------------------------
beam_vis = fea.ChVisualShapeFEA(mesh)
beam_vis.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
beam_vis.SetColorscaleMinMax(-50.0, 50.0)
beam_vis.SetSmoothFaces(True)
beam_vis.SetWireframe(False)
mesh.AddVisualShapeFEA(beam_vis)

node_vis = fea.ChVisualShapeFEA(mesh)
node_vis.SetFEMglyphType(fea.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
node_vis.SetFEMdataType(fea.ChVisualShapeFEA.DataType_NONE)
node_vis.SetSymbolsThickness(0.004)
node_vis.SetSymbolsScale(0.002)
node_vis.SetZbufferHide(False)
mesh.AddVisualShapeFEA(node_vis)


# -----------------------------------------------------------------------------
# Solver and timestepper
# -----------------------------------------------------------------------------
solver = chrono.ChSolverMINRES()
solver.SetMaxIterations(300)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.SetVerbose(False)
system.SetSolver(solver)

# HHT is suitable for stiff FEA dynamics and gives numerical damping.
try:
    timestepper = chrono.ChTimestepperHHT(system)
    timestepper.SetAlpha(-0.20)

    if hasattr(timestepper, "SetMaxIters"):
        timestepper.SetMaxIters(20)
    elif hasattr(timestepper, "SetMaxiters"):
        timestepper.SetMaxiters(20)

    try:
        timestepper.SetAbsTolerances(1e-8, 1e-8)
    except TypeError:
        timestepper.SetAbsTolerances(1e-8)

    timestepper.SetVerbose(False)
    system.SetTimestepper(timestepper)

except Exception:
    # Fallback for older PyChrono builds
    system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)


# -----------------------------------------------------------------------------
# Irrlicht visualization
# -----------------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono FEA Beam Buckling with Motor-Driven Compression")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(Vec(0.50, 0.35, -1.25), Vec(0.50, 0.0, 0.0))
vis.AddTypicalLights()

try:
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
except Exception:
    pass


# -----------------------------------------------------------------------------
# Simulation loop
# -----------------------------------------------------------------------------
dt = 1.0e-4
render_every = 10

perturb = LateralPerturbationFunction(force=0.20, duration=0.20)
mid_node = beam_nodes[len(beam_nodes) // 2]

step = 0

while vis.Run():
    time = system.GetChTime()

    # Apply a short lateral force pulse at the beam midpoint.
    # This helps select a buckling direction.
    lateral_force = perturb.GetVal(time)
    try:
        mid_node.SetForce(Vec(0, lateral_force, 0))
    except Exception:
        pass

    if step % render_every == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    system.DoStepDynamics(dt)
    step += 1