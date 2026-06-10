"""NSC gear-and-pulley multibody demo.

This standalone PyChrono model builds a fixed truss, a driven gear A, a bevel
gear D at a right-angle shaft, and pulley E coupled to gear D by a synchronous
belt constraint. Gear A drives gear D with a 1:1 gear relation, and the belt
transfers D's rotation to the smaller pulley while simplified belt spans remain
visible in the Irrlicht scene.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants ===
TIME_STEP = 1e-3
SIM_END = 8.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

GEAR_A_RADIUS = 5.0
GEAR_D_RADIUS = 5.0
PULLEY_E_RADIUS = 2.0
GEAR_THICKNESS = 0.8
PULLEY_THICKNESS = 0.7
DENSITY = 1000.0
MOTOR_SPEED = 0.6

GEAR_A_POS = chrono.ChVector3d(0.0, 0.0, -9.0)
GEAR_D_POS = chrono.ChVector3d(-10.0, 0.0, -9.0)
PULLEY_E_POS = chrono.ChVector3d(-10.0, -11.0, -9.0)
SHAFT_Y = chrono.QuatFromAngleX(-chrono.CH_PI_2)
SHAFT_X = chrono.Q_ROTATE_Z_TO_X


# === Helper functions ===
def color_shape(body, color):
    shape = body.GetVisualShape(0)  # cache: visual shape fetched once per body
    shape.SetColor(color)


def add_belt_span(anchor, p1, p2, color):
    span = chrono.ChVisualShapeLine()
    span.SetLineGeometry(chrono.ChLineSegment(p1, p2))
    span.SetThickness(0.22)
    span.SetColor(color)
    anchor.AddVisualShape(span)


def add_radial_spokes(body, radius, color, count):
    for i in range(count):
        angle = 2.0 * math.pi * i / count
        rim = chrono.ChVector3d(radius * math.cos(angle), -0.46, radius * math.sin(angle))
        spoke = chrono.ChVisualShapeLine()
        spoke.SetLineGeometry(chrono.ChLineSegment(chrono.ChVector3d(0.0, -0.46, 0.0), rim))
        spoke.SetThickness(0.12)
        spoke.SetColor(color)
        body.AddVisualShape(spoke)


def add_face_ring(body, radius, color, count):
    for i in range(count):
        a0 = 2.0 * math.pi * i / count
        a1 = 2.0 * math.pi * (i + 1) / count
        p0 = chrono.ChVector3d(radius * math.cos(a0), -0.48, radius * math.sin(a0))
        p1 = chrono.ChVector3d(radius * math.cos(a1), -0.48, radius * math.sin(a1))
        edge = chrono.ChVisualShapeLine()
        edge.SetLineGeometry(chrono.ChLineSegment(p0, p1))
        edge.SetThickness(0.14)
        edge.SetColor(color)
        body.AddVisualShape(edge)


def add_simple_teeth(body, radius, color, count):
    tooth_size = chrono.ChVector3d(0.35, 0.55, 0.18)
    for i in range(count):
        angle = 2.0 * math.pi * i / count
        pos = chrono.ChVector3d(radius * math.cos(angle), 0.0, radius * math.sin(angle))
        tooth = chrono.ChVisualShapeBox(tooth_size)
        tooth.SetColor(color)
        body.AddVisualShape(tooth, chrono.ChFramed(pos, chrono.QuatFromAngleY(-angle)))


# === System & fixed truss ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

truss = chrono.ChBody()
truss.SetName("fixed truss")
truss.SetFixed(True)
truss.EnableCollision(False)
sys.AddBody(truss)

support = chrono.ChBodyEasyBox(22.0, 0.25, 0.25, DENSITY)
support.SetName("truss beam")
support.SetPos(chrono.ChVector3d(-5.0, 0.0, -9.0))
support.SetFixed(True)
color_shape(support, chrono.ChColor(0.35, 0.35, 0.38))
sys.AddBody(support)


# === Bodies ===
gear_a = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, GEAR_A_RADIUS, GEAR_THICKNESS, DENSITY)
gear_a.SetName("gear A")
gear_a.SetPos(GEAR_A_POS)
gear_a.SetRot(chrono.QUNIT)
color_shape(gear_a, chrono.ChColor(0.15, 0.42, 0.82))
add_radial_spokes(gear_a, GEAR_A_RADIUS * 0.92, chrono.ChColor(1.0, 1.0, 1.0), 8)
add_face_ring(gear_a, GEAR_A_RADIUS * 0.92, chrono.ChColor(1.0, 1.0, 1.0), 36)
add_simple_teeth(gear_a, GEAR_A_RADIUS * 1.02, chrono.ChColor(0.08, 0.22, 0.45), 20)
sys.AddBody(gear_a)

gear_d = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, GEAR_D_RADIUS, GEAR_THICKNESS, DENSITY)
gear_d.SetName("bevel gear D")
gear_d.SetPos(GEAR_D_POS)
gear_d.SetRot(chrono.QuatFromAngleZ(chrono.CH_PI_2))
color_shape(gear_d, chrono.ChColor(0.86, 0.35, 0.16))
add_radial_spokes(gear_d, GEAR_D_RADIUS * 0.92, chrono.ChColor(1.0, 0.95, 0.8), 8)
add_face_ring(gear_d, GEAR_D_RADIUS * 0.92, chrono.ChColor(1.0, 0.95, 0.8), 36)
add_simple_teeth(gear_d, GEAR_D_RADIUS * 1.02, chrono.ChColor(0.45, 0.12, 0.04), 20)
sys.AddBody(gear_d)

pulley_e = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, PULLEY_E_RADIUS, PULLEY_THICKNESS, DENSITY)
pulley_e.SetName("pulley E")
pulley_e.SetPos(PULLEY_E_POS)
pulley_e.SetRot(chrono.QuatFromAngleZ(chrono.CH_PI_2))
color_shape(pulley_e, chrono.ChColor(0.22, 0.68, 0.38))
add_radial_spokes(pulley_e, PULLEY_E_RADIUS * 0.85, chrono.ChColor(0.9, 1.0, 0.9), 6)
add_face_ring(pulley_e, PULLEY_E_RADIUS * 0.85, chrono.ChColor(0.9, 1.0, 0.9), 24)
sys.AddBody(pulley_e)


# === Joints / constraints ===
motor_a = chrono.ChLinkMotorRotationSpeed()
motor_a.SetName("gear A speed motor")
motor_a.Initialize(gear_a, truss, chrono.ChFramed(GEAR_A_POS, SHAFT_Y))
motor_a.SetSpeedFunction(chrono.ChFunctionConst(MOTOR_SPEED))
sys.AddLink(motor_a)

rev_d = chrono.ChLinkLockRevolute()
rev_d.SetName("gear D horizontal revolute")
rev_d.Initialize(gear_d, truss, chrono.ChFramed(GEAR_D_POS, SHAFT_X))
sys.AddLink(rev_d)

rev_e = chrono.ChLinkLockRevolute()
rev_e.SetName("pulley E horizontal revolute")
rev_e.Initialize(pulley_e, truss, chrono.ChFramed(PULLEY_E_POS, SHAFT_X))
sys.AddLink(rev_e)

gear_ad = chrono.ChLinkLockGear()
gear_ad.SetName("gear A to bevel gear D 1:1")
gear_ad.Initialize(gear_a, gear_d, chrono.ChFramed())
gear_ad.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, SHAFT_Y))
gear_ad.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, SHAFT_X))
gear_ad.SetTransmissionRatio(1.0)
gear_ad.SetEnforcePhase(True)
sys.AddLink(gear_ad)

belt_de = chrono.ChLinkLockPulley()
belt_de.SetName("synchro belt D to E")
belt_de.Initialize(gear_d, pulley_e, chrono.ChFramed())
belt_de.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, SHAFT_X))
belt_de.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, SHAFT_X))
belt_de.SetRadius1(GEAR_D_RADIUS)
belt_de.SetRadius2(PULLEY_E_RADIUS)
belt_de.SetEnforcePhase(True)
sys.AddLink(belt_de)


# === Belt visualization ===
belt_color = chrono.ChColor(0.03, 0.03, 0.03)
add_belt_span(truss, chrono.ChVector3d(-10.0, 0.0, -4.0), chrono.ChVector3d(-10.0, -11.0, -7.0), belt_color)
add_belt_span(truss, chrono.ChVector3d(-10.0, 0.0, -14.0), chrono.ChVector3d(-10.0, -11.0, -11.0), belt_color)


# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Gear A, Bevel Gear D, and Pulley E")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-24, -24, 2), chrono.ChVector3d(-9, -5, -9))
vis.AddTypicalLights()
vis.AddLight(chrono.ChVector3d(-18, -20, 6), 80, chrono.ChColor(1.0, 1.0, 1.0))


# === Main loop ===
frame = 0


try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sim_time = sys.GetChTime()  # cache: reused for review logging
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:
    traceback.print_exc()
    raise
finally:
    pass
