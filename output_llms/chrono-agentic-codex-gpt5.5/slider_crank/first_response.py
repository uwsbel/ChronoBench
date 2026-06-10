"""Motor-driven slider-crank mechanism using a PyChrono NSC system.

The model contains a fixed floor/truss, a crankshaft body, a connecting rod,
and a piston. A prescribed-speed rotational motor turns the crankshaft about
the Z axis, while revolute pin joints and a prismatic guide convert the rotation
into horizontal piston motion.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants ===
TIME_STEP = 1.0e-3
SIM_END = 6.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

CRANK_RADIUS = 0.35
ROD_LENGTH = 1.25
MOTOR_SPEED = 2.0 * chrono.CH_PI

TRUSS_SIZE = chrono.ChVector3d(2.4, 0.12, 0.12)
PISTON_SIZE = chrono.ChVector3d(0.28, 0.22, 0.24)
LINK_RADIUS = 0.045
PIN_RADIUS = 0.07
DISC_RADIUS = 0.18
DISC_WIDTH = 0.08

CRANK_CENTER = chrono.ChVector3d(0.0, 0.0, 0.0)
CRANK_PIN = chrono.ChVector3d(CRANK_RADIUS, 0.0, 0.0)
PISTON_CENTER = chrono.ChVector3d(CRANK_RADIUS + ROD_LENGTH, 0.0, 0.0)
ROD_CENTER = chrono.ChVector3d((CRANK_PIN.x + PISTON_CENTER.x) * 0.5, 0.0, 0.0)


# === Small helpers ===
def color_shape(shape, r, g, b):
    """Apply an RGB color to a visual shape and return it."""
    shape.SetColor(chrono.ChColor(r, g, b))
    return shape


def add_box_visual(body, size, color, frame=chrono.ChFramed()):
    """Attach one box visual to a body."""
    box = chrono.ChVisualShapeBox(size)
    box.SetColor(chrono.ChColor(*color))
    body.AddVisualShape(box, frame)


def add_link_cylinder(body, radius, length, color, frame=chrono.ChFramed()):
    """Attach a cylinder whose local axis is body X."""
    cyl = chrono.ChVisualShapeCylinder(radius, length)
    cyl.SetColor(chrono.ChColor(*color))
    body.AddVisualShape(
        cyl,
        chrono.ChFramed(frame.GetPos(), frame.GetRot() * chrono.QuatFromAngleY(chrono.CH_PI_2)),
    )


# === System & gravity ===
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))


# === Bodies ===
truss = chrono.ChBody()
truss.SetName("fixed floor truss")
truss.SetFixed(True)
truss.SetPos(chrono.ChVector3d(0.85, -0.28, 0.0))
add_box_visual(truss, TRUSS_SIZE, (0.42, 0.42, 0.42))
add_link_cylinder(
    truss,
    0.035,
    0.36,
    (0.18, 0.18, 0.18),
    chrono.ChFramed(CRANK_CENTER - truss.GetPos(), chrono.QUNIT),
)
system.AddBody(truss)

crank = chrono.ChBody()
crank.SetName("crankshaft and crank arm")
crank.SetMass(1.0)
crank.SetInertiaXX(chrono.ChVector3d(0.025, 0.025, 0.025))
crank.SetPos(CRANK_CENTER)
crank.SetRot(chrono.QUNIT)
crank.EnableCollision(False)
crank_disc = color_shape(chrono.ChVisualShapeCylinder(DISC_RADIUS, DISC_WIDTH), 0.2, 0.35, 0.85)
crank.AddVisualShape(crank_disc, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))
add_link_cylinder(
    crank,
    LINK_RADIUS,
    CRANK_RADIUS,
    (0.88, 0.25, 0.18),
    chrono.ChFramed(chrono.ChVector3d(CRANK_RADIUS * 0.5, 0.0, 0.0), chrono.QUNIT),
)
pin_shape = chrono.ChVisualShapeSphere(PIN_RADIUS)
pin_shape.SetColor(chrono.ChColor(0.92, 0.74, 0.22))
crank.AddVisualShape(pin_shape, chrono.ChFramed(chrono.ChVector3d(CRANK_RADIUS, 0.0, 0.0)))
system.AddBody(crank)

rod = chrono.ChBody()
rod.SetName("connecting rod")
rod.SetMass(0.8)
rod.SetInertiaXX(chrono.ChVector3d(0.08, 0.08, 0.08))
rod.SetPos(ROD_CENTER)
rod.SetRot(chrono.QUNIT)
rod.EnableCollision(False)
add_link_cylinder(rod, LINK_RADIUS, ROD_LENGTH, (0.1, 0.65, 0.32))
system.AddBody(rod)

piston = chrono.ChBody()
piston.SetName("piston slider")
piston.SetMass(1.0)
piston.SetInertiaXX(chrono.ChVector3d(0.02, 0.02, 0.02))
piston.SetPos(PISTON_CENTER)
piston.SetRot(chrono.QUNIT)
piston.EnableCollision(False)
add_box_visual(piston, PISTON_SIZE, (0.75, 0.2, 0.55))
wrist_pin = chrono.ChVisualShapeSphere(PIN_RADIUS)
wrist_pin.SetColor(chrono.ChColor(0.92, 0.74, 0.22))
piston.AddVisualShape(wrist_pin)
system.AddBody(piston)


# === Joints / constraints ===
crank_motor = chrono.ChLinkMotorRotationSpeed()
crank_motor.Initialize(crank, truss, chrono.ChFramed(CRANK_CENTER, chrono.QUNIT))
crank_motor.SetSpeedFunction(chrono.ChFunctionConst(MOTOR_SPEED))
system.AddLink(crank_motor)

crank_rod = chrono.ChLinkLockRevolute()
crank_rod.Initialize(
    crank,
    rod,
    True,
    chrono.ChFramed(chrono.ChVector3d(CRANK_RADIUS, 0.0, 0.0), chrono.QUNIT),
    chrono.ChFramed(chrono.ChVector3d(-ROD_LENGTH * 0.5, 0.0, 0.0), chrono.QUNIT),
)
system.AddLink(crank_rod)

rod_piston = chrono.ChLinkLockRevolute()
rod_piston.Initialize(
    rod,
    piston,
    True,
    chrono.ChFramed(chrono.ChVector3d(ROD_LENGTH * 0.5, 0.0, 0.0), chrono.QUNIT),
    chrono.ChFramed(chrono.VNULL, chrono.QUNIT),
)
system.AddLink(rod_piston)

piston_guide = chrono.ChLinkLockPrismatic()
piston_guide.Initialize(piston, truss, chrono.ChFramed(PISTON_CENTER, chrono.Q_ROTATE_Z_TO_X))
system.AddLink(piston_guide)


# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono Slider-Crank")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.3, -2.8, 1.2), chrono.ChVector3d(0.75, 0.0, 0.0))
vis.AddTypicalLights()
vis.AddGrid(
    0.2,
    0.2,
    14,
    8,
    chrono.ChCoordsysd(chrono.ChVector3d(0.8, -0.36, 0.0), chrono.Q_ROTATE_Z_TO_Y),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main loop ===

try:

    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            system.DoStepDynamics(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (OSError, IOError) as exc:  # visualization device or output errors
    traceback.print_exc()
    raise
except (RuntimeError, ValueError) as exc:  # solver or invalid-state errors
    traceback.print_exc()
    raise
finally:
    pass


# === Post-processing ===
