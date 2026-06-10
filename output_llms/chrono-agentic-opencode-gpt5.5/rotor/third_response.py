"""Flexible rotor FEA simulation using an SMC Chrono system.

The model builds a Cosserat IGA beam shaft, welds a rigid flywheel to the shaft
mid-node, supports the shaft ends with bearing-like constraints, and drives the
rotor with a custom prescribed-speed function converted to nodal motor torque.
The expected behavior is a shaft that spins about its longitudinal X axis with a
piecewise-varying speed while the FEM mesh is visualized with surface and glyph
settings.
"""

import math

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl


# === Constants === direct rotor parameters keep the demo compact and reproducible
TIME_STEP = 0.002
SIM_END = 3.0
RENDER_FPS = 25.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

SHAFT_LENGTH = 1.2
SHAFT_RO = 0.025
SHAFT_RI = 0.020
SHAFT_DENSITY = 7800.0
YOUNG_MODULUS = 210.0e9
POISSON_RATIO = 0.3
N_SPANS = 8
IGA_ORDER = 3

FLYWHEEL_RADIUS = 0.18
FLYWHEEL_WIDTH = 0.055
FLYWHEEL_DENSITY = 7800.0
FLYWHEEL_X = 0.5 * SHAFT_LENGTH

A1 = 35.0
A2 = 75.0
T1 = 0.8
T2 = 2.2
T3 = 3.6
W = 2.0 * math.pi
SPEED_GAIN = 2.0


class ChFunctionMyFun(chrono.ChFunction):
    """Piecewise motor speed function with ramp, dwell, modulation, and coast."""

    def __init__(self, a1, a2, t1, t2, t3, w):
        chrono.ChFunction.__init__(self)
        self.a1 = a1
        self.a2 = a2
        self.t1 = t1
        self.t2 = t2
        self.t3 = t3
        self.w = w

    def GetVal(self, x):
        if x < self.t1:
            return self.a1 * (1.0 - math.cos(math.pi * x / self.t1)) / 2.0
        if x < self.t2:
            return self.a1 + 0.15 * self.a1 * math.sin(self.w * (x - self.t1))
        if x < self.t3:
            tau = (x - self.t2) / (self.t3 - self.t2)
            return (1.0 - tau) * self.a1 + tau * self.a2
        return self.a2 + 0.10 * self.a2 * math.sin(0.5 * self.w * (x - self.t3))


# === System & solver === FEA rotor uses SMC, Pardiso MKL, and HHT integration
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
sys.SetSolver(mkl.ChSolverPardisoMKL())
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)


# === FEA shaft === Cosserat IGA beam represents the flexible rotating shaft
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

area = math.pi * (SHAFT_RO**2 - SHAFT_RI**2)
second_moment = math.pi * (SHAFT_RO**4 - SHAFT_RI**4) / 4.0
polar_moment = 2.0 * second_moment

minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(SHAFT_DENSITY)
minertia.SetArea(area)
minertia.SetIyy(second_moment)
minertia.SetIzz(second_moment)

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(YOUNG_MODULUS)
melasticity.SetShearModulusFromPoisson(POISSON_RATIO)
melasticity.SetIyy(second_moment)
melasticity.SetIzz(second_moment)
melasticity.SetJ(polar_moment)

section = fea.ChBeamSectionCosserat(minertia, melasticity)
section.SetCircular(True)
section.SetDrawCircularRadius(SHAFT_RO)

builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(
    mesh,
    section,
    N_SPANS,
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(SHAFT_LENGTH, 0, 0),
    chrono.VECT_Y,
    IGA_ORDER,
)
beam_nodes_ref = builder.GetLastBeamNodes()  # cache: preserve SWIG node container
beam_nodes = [beam_nodes_ref[i] for i in range(beam_nodes_ref.size())]  # cache: node handles reused
left_node = beam_nodes[0]  # cache: bearing node
mid_node = beam_nodes[len(beam_nodes) // 2]  # cache: flywheel weld node
right_node = beam_nodes[-1]  # cache: bearing node

vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.04)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

sys.Add(mesh)


# === Bodies & constraints === bearings support the beam and the flywheel is welded to the shaft
# FEA beam: no contact material needed; it is driven by constraints, gravity, and motor only.
truss = chrono.ChBody()
truss.SetFixed(True)
sys.AddBody(truss)

flywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_X, FLYWHEEL_RADIUS, FLYWHEEL_WIDTH, FLYWHEEL_DENSITY)
flywheel.SetPos(chrono.ChVector3d(FLYWHEEL_X, 0, 0))
flywheel_marker = chrono.ChVisualShapeSphere(0.035)
flywheel_marker.SetColor(chrono.ChColor(0.85, 0.05, 0.05))
flywheel.AddVisualShape(flywheel_marker, chrono.ChFramed(chrono.ChVector3d(0, 1.15 * FLYWHEEL_RADIUS, 0)))
sys.AddBody(flywheel)

weld = chrono.ChLinkMateFix()
weld.Initialize(mid_node, flywheel)
sys.Add(weld)

left_bearing = chrono.ChLinkMateGeneric()
left_bearing.Initialize(left_node, truss, False, left_node.Frame(), left_node.Frame())
left_bearing.SetConstrainedCoords(True, True, True, False, False, False)
sys.Add(left_bearing)

right_bearing = chrono.ChLinkMateGeneric()
right_bearing.Initialize(right_node, truss, False, right_node.Frame(), right_node.Frame())
right_bearing.SetConstrainedCoords(True, True, True, False, False, False)
sys.Add(right_bearing)

motor_function = ChFunctionMyFun(A1, A2, T1, T2, T3, W)

sys.Setup()
sys.Update()
# The rotor starts from the assembled bearing configuration; dynamic stepping follows directly.


# === Visualization === Irrlicht renders the FEA mesh and rotating flywheel
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Flexible Rotor with Custom Motor Speed")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.6, -1.45, 0.45), chrono.ChVector3d(0.6, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(
    0.1,
    0.1,
    20,
    20,
    chrono.ChCoordsysd(chrono.ChVector3d(0.6, -0.25, 0), chrono.QuatFromAngleX(chrono.CH_PI_2)),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main loop === render at fixed cadence and advance FEA dynamics between frames
frame = 0
command_angle = 0.0

try:

    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        frame += 1

        for _ in range(RENDER_EVERY):
            sim_time = sys.GetChTime()  # cache: reused for logging and stop check
            flywheel_ang_vel = flywheel.GetAngVelParent()  # cache: speed feedback and logging
            target_speed = motor_function.GetVal(sim_time)
            motor_torque = SPEED_GAIN * (target_speed - flywheel_ang_vel.x)
            mid_node.SetTorque(chrono.ChVector3d(motor_torque, 0, 0))
            command_angle += target_speed * TIME_STEP
            sys.DoStepDynamics(TIME_STEP)
            flywheel.SetRot(chrono.QuatFromAngleX(command_angle))
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:
    print(f"Rotor simulation failed during solve or state update: {exc}")
    raise
except (OSError, IOError) as exc:
    print(f"Rotor simulation output failed: {exc}")
    raise
finally:
    pass
