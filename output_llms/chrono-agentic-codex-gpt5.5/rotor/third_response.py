"""Flexible IGA rotor driven by a custom piecewise speed function.

The model uses a Chrono SMC FEA system with a Cosserat shaft, bearing
constraints, and a rigid flywheel welded to the shaft midspan.  A
ChLinkMotorRotationSpeed applies the requested ChFunctionMyFun speed profile,
so the rotor accelerates through several operating regimes while the FEA mesh
is visualized with surface and glyph settings.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl


# === Constants ===
TIME_STEP = 0.002
SIM_END = 3.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

SHAFT_LENGTH = 6.0
SHAFT_RADIUS_OUTER = 0.055
SHAFT_RADIUS_INNER = 0.035
SHAFT_DENSITY = 7800.0
YOUNG_MODULUS = 2.1e11
POISSON_RATIO = 0.30
IGA_SPANS = 16
IGA_ORDER = 3

FLYWHEEL_RADIUS = 0.42
FLYWHEEL_WIDTH = 0.20
FLYWHEEL_DENSITY = 7800.0
FLYWHEEL_POS = chrono.ChVector3d(SHAFT_LENGTH * 0.5, 0.0, 0.0)

SPEED_A1 = 18.0
SPEED_A2 = 42.0
SPEED_T1 = 0.45
SPEED_T2 = 1.25
SPEED_T3 = 2.15
SPEED_W = 2.0 * chrono.CH_PI


class ChFunctionMyFun(chrono.ChFunction):
    """Piecewise motor-speed function using A1, A2, T1, T2, T3, and w."""

    def __init__(self, A1, A2, T1, T2, T3, w):
        chrono.ChFunction.__init__(self)
        self.A1 = A1
        self.A2 = A2
        self.T1 = T1
        self.T2 = T2
        self.T3 = T3
        self.w = w

    def GetVal(self, x):
        if x < self.T1:
            blend = 0.5 * (1.0 - math.cos(math.pi * x / self.T1))
            return self.A1 * blend
        if x < self.T2:
            return self.A1 + 0.12 * self.A1 * math.sin(self.w * (x - self.T1))
        if x < self.T3:
            tau = (x - self.T2) / (self.T3 - self.T2)
            return self.A1 + (self.A2 - self.A1) * (0.5 - 0.5 * math.cos(math.pi * tau))
        return self.A2 + 0.08 * self.A2 * math.sin(0.5 * self.w * (x - self.T3))


# === System and solver ===
system = chrono.ChSystemSMC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))
system.SetSolver(mkl.ChSolverPardisoMKL())
timestepper = chrono.ChTimestepperHHT(system)
timestepper.SetStepControl(False)
system.SetTimestepper(timestepper)


# === FEA shaft ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

area = math.pi * (SHAFT_RADIUS_OUTER**2 - SHAFT_RADIUS_INNER**2)
iyy = math.pi * (SHAFT_RADIUS_OUTER**4 - SHAFT_RADIUS_INNER**4) / 4.0
izz = iyy
jpolar = 2.0 * iyy

inertia = fea.ChInertiaCosseratSimple()
inertia.SetDensity(SHAFT_DENSITY)
inertia.SetArea(area)
inertia.SetIyy(iyy)
inertia.SetIzz(izz)

elasticity = fea.ChElasticityCosseratSimple()
elasticity.SetYoungModulus(YOUNG_MODULUS)
elasticity.SetShearModulusFromPoisson(POISSON_RATIO)
elasticity.SetIyy(iyy)
elasticity.SetIzz(izz)
elasticity.SetJ(jpolar)

section = fea.ChBeamSectionCosserat(inertia, elasticity)
section.SetCircular(True)
section.SetDrawCircularRadius(SHAFT_RADIUS_OUTER)

builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(
    mesh,
    section,
    IGA_SPANS,
    chrono.ChVector3d(0.0, 0.0, 0.0),
    chrono.ChVector3d(SHAFT_LENGTH, 0.0, 0.0),
    chrono.VECT_Y,
    IGA_ORDER,
)
beam_node_container = builder.GetLastBeamNodes()  # cache: keep SWIG container alive
beam_nodes = [beam_node_container[i] for i in range(beam_node_container.size())]  # cache: node refs
left_node = beam_nodes[0]  # cache: bearing node
mid_node = beam_nodes[len(beam_nodes) // 2]  # cache: flywheel node
right_node = beam_nodes[-1]  # cache: bearing node

# FEA beam: no contact material needed; driven by constraints and a motor only.
surface = chrono.ChVisualShapeFEA(mesh)
surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
surface.SetSmoothFaces(True)
surface.SetWireframe(False)
mesh.AddVisualShapeFEA(surface)

glyph = chrono.ChVisualShapeFEA(mesh)
glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
glyph.SetSymbolsThickness(0.006)
glyph.SetSymbolsScale(0.01)
glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(glyph)

system.Add(mesh)


# === Rigid bodies and constraints ===
truss = chrono.ChBody()
truss.SetFixed(True)
system.AddBody(truss)

flywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_X, FLYWHEEL_RADIUS, FLYWHEEL_WIDTH, FLYWHEEL_DENSITY)
flywheel.SetPos(FLYWHEEL_POS)
flywheel.EnableCollision(False)
system.AddBody(flywheel)

weld_flywheel = chrono.ChLinkMateFix()
weld_flywheel.Initialize(mid_node, flywheel)
system.Add(weld_flywheel)

left_bearing = chrono.ChLinkMateGeneric()
left_bearing.Initialize(left_node, truss, False, left_node.Frame(), left_node.Frame())
left_bearing.SetConstrainedCoords(True, True, True, False, True, True)
system.Add(left_bearing)

right_bearing = chrono.ChLinkMateGeneric()
right_bearing.Initialize(right_node, truss, False, right_node.Frame(), right_node.Frame())
right_bearing.SetConstrainedCoords(True, True, True, False, True, True)
system.Add(right_bearing)

motor_frame = chrono.ChFramed(FLYWHEEL_POS, chrono.Q_ROTATE_Z_TO_X)
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(flywheel, truss, motor_frame)
motor_speed_function = ChFunctionMyFun(SPEED_A1, SPEED_A2, SPEED_T1, SPEED_T2, SPEED_T3, SPEED_W)
motor.SetSpeedFunction(motor_speed_function)
system.AddLink(motor)

system.Setup()
system.Update()


# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Custom speed flexible rotor")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3.0, 2.0, 7.0), chrono.ChVector3d(3.0, 0.0, 0.0))
vis.AddTypicalLights()


# === Main loop ===
def run_simulation():
    frame = 0
    try:
        while vis.Run() and system.GetChTime() < SIM_END:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            frame += 1
            for _ in range(RENDER_EVERY):
                system.DoStepDynamics(TIME_STEP)
                if system.GetChTime() >= SIM_END:
                    break
    except (RuntimeError, ValueError) as exc:  # solver divergence / invalid state
        traceback.print_exc()
        raise
    finally:
        pass


if __name__ == "__main__":
    run_simulation()
